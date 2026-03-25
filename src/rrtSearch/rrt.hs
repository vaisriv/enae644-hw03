module RRT
  ( -- Types (re-exported from Types module)
    module Types,
    Problem (..),
    -- Geometry
    distPoint,
    segmentClearOfObstacle,
    segmentClear,
    inGoal,
    inWorkspace,
    -- Tree operations
    nearestNode,
    extendToward,
    -- Algorithm
    rrt,
  )
where

import Control.Monad (when)
import qualified Data.List (minimumBy)
import qualified Data.Map.Strict as Map
import Data.Ord (comparing)
import qualified Dynamics
  ( RobotShape (..),
    State5D (..),
    Trajectory,
    ValidationFailure (..),
    distState5D,
    robotBoundingRadius,
    steer,
    validateFinalPath,
    validateFinalPathWithDiagnostics,
  )
import qualified Graphs
  ( Graph (..),
    Node (..),
    addNode,
    emptyGraph,
    graphNodes,
    pathToRoot,
  )
import System.IO (hFlush, stdout)
import System.Random (RandomGen, randomR)
import Types

-------------------------------------------------------------------------------
-- Types
-------------------------------------------------------------------------------

-- | a fully-specified RRT problem instance
data Problem = Problem
  { problemWorkspace :: Workspace,
    problemStart :: (Double, Double, Double, Double, Double), -- (x, y, theta, v, w)
    problemGoal :: Goal,
    problemMotion :: Motion,
    problemObstacles :: [Obstacle],
    problemRobot :: Dynamics.RobotShape
  }
  deriving (Show)

-------------------------------------------------------------------------------
-- Geometry
-------------------------------------------------------------------------------

-- | euclidean distance between two points
distPoint :: (Double, Double) -> (Double, Double) -> Double
distPoint (x1, y1) (x2, y2) = sqrt ((x2 - x1) ^ (2 :: Int) + (y2 - y1) ^ (2 :: Int))

-- | test whether line segment [p1->p2] is clear of a single obstacle
--   use closest-point-on-segment geometry:
--     1. project the obstacle centre onto the segment
--     2. clamp to [0,1]
--     3. compare distance to radius
segmentClearOfObstacle :: (Double, Double) -> (Double, Double) -> Obstacle -> Bool
segmentClearOfObstacle (x1, y1) (x2, y2) obs =
  let cx = obstacleX obs
      cy = obstacleY obs
      r = obstacleRadius obs
      -- segment vector
      dx = x2 - x1
      dy = y2 - y1
      -- vector from segment start to circle centre
      fx = x1 - cx
      fy = y1 - cy
      a = dx * dx + dy * dy
      b = 2 * (fx * dx + fy * dy)
      c = fx * fx + fy * fy - r * r
      discriminant = b * b - 4 * a * c
   in if discriminant < 0
        then True -- no intersection with infinite line
        else
          let sqrtD = sqrt discriminant
              t1 = (-b - sqrtD) / (2 * a)
              t2 = (-b + sqrtD) / (2 * a)
           in -- segment is clear if both intersections are outside [0, 1]
              not ((t1 >= 0 && t1 <= 1) || (t2 >= 0 && t2 <= 1) || (t1 <= 0 && t2 >= 1))

-- | test whether a segment is clear of ALL obstacles
segmentClear :: (Double, Double) -> (Double, Double) -> [Obstacle] -> Bool
segmentClear p1 p2 = all (segmentClearOfObstacle p1 p2)

-- | test whether a point lies within the GOAL region
inGoal :: (Double, Double) -> Goal -> Bool
inGoal (x, y) g = distPoint (x, y) (goalX g, goalY g) <= goalRadius g

-- | test whether a point lies within the WORKSPACE bounds
inWorkspace :: (Double, Double) -> Workspace -> Bool
inWorkspace (x, y) ws =
  x >= workspaceXMin ws
    && x <= workspaceXMax ws
    && y >= workspaceYMin ws
    && y <= workspaceYMax ws

-------------------------------------------------------------------------------
-- Tree operations
-------------------------------------------------------------------------------

-- | find the nearest node in the graph to a query 5D state
nearestNode5D :: Dynamics.State5D -> Graphs.Graph -> Graphs.Node
nearestNode5D qstate g =
  Data.List.minimumBy
    (comparing (\n ->
      let nstate = Dynamics.State5D (Graphs.nodeX n) (Graphs.nodeY n) (Graphs.nodeTheta n) (Graphs.nodeV n) (Graphs.nodeW n)
       in Dynamics.distState5D qstate nstate))
    (Map.elems (Graphs.graphNodes g))

-- | find the nearest node in the graph to a query point (legacy 2D version)
nearestNode :: (Double, Double) -> Graphs.Graph -> Graphs.Node
nearestNode (qx, qy) g =
  Data.List.minimumBy
    (comparing (\n -> distPoint (qx, qy) (Graphs.nodeX n, Graphs.nodeY n)))
    (Map.elems (Graphs.graphNodes g))

-- | extend from a node toward a target point by at most epsilon
--   returns the new point (clamped to epsilon distance if target is farther)
extendToward :: (Double, Double) -> (Double, Double) -> Double -> (Double, Double)
extendToward (fx, fy) (tx, ty) epsilon =
  let d = distPoint (fx, fy) (tx, ty)
   in if d <= epsilon
        then (tx, ty)
        else
          let scale = epsilon / d
           in (fx + scale * (tx - fx), fy + scale * (ty - fy))

-------------------------------------------------------------------------------
-- RRT algorithm
-------------------------------------------------------------------------------

-- | run RRT with progress reporting
--   returns the final graph (search tree) and either the path from
--   start to goal (Right) or an error message (Left) if max iterations reached
--
--   parameters:
--     problem : fully-specified problem instance
--     maxIter : iteration cap
--     gen     : random generator (pure — no IO)
rrt ::
  (RandomGen g) =>
  Problem ->
  Int ->
  g ->
  IO (Graphs.Graph, Either String [Graphs.Node])
rrt problem maxIter gen = do
  let (sx, sy, stheta, sv, sw) = problemStart problem
      (initGraph, _) = Graphs.addNode sx sy stheta sv sw Nothing Nothing Nothing Graphs.emptyGraph
  putStrLn "RRT search started..."
  go initGraph gen 0 0
  where
    reportInterval = 500  -- Report every 500 iterations
    ws = problemWorkspace problem
    goal = problemGoal problem
    obs = problemObstacles problem
    robot = problemRobot problem
    eps = motionEpsilon (problemMotion problem)

    go g gen' iter steerSuccesses
      | iter >= maxIter = do
          putStrLn $ "\nSearch exhausted: " ++ show maxIter ++ " iterations"
          return
            ( g,
              Left $ "No path found after " ++ show maxIter ++ " iterations"
            )
      | otherwise = do
          -- Progress reporting every reportInterval iterations
          when (iter > 0 && iter `mod` reportInterval == 0) $ do
            let nodes = Map.size (Graphs.graphNodes g)
                successRate = if iter > 0 then (100.0 * fromIntegral steerSuccesses / fromIntegral iter :: Double) else 0.0
            putStr $ "\rIter: " ++ show iter ++ "/" ++ show maxIter
                  ++ " | Nodes: " ++ show nodes
                  ++ " | Steering success: " ++ show (round successRate :: Int) ++ "%"
            hFlush stdout
          -- 1. sample a random 5D state (10% goal biasing)
          let (biasRoll, gen1) = randomR (0.0 :: Double, 1.0 :: Double) gen'
              (qrand, gen2) = if biasRoll < 0.1
                              then sampleGoalState gen1
                              else sampleRandomState gen1
              -- 2. find nearest node in tree using 5D distance
              nearest = nearestNode5D qrand g
              nearestState = Dynamics.State5D
                (Graphs.nodeX nearest)
                (Graphs.nodeY nearest)
                (Graphs.nodeTheta nearest)
                (Graphs.nodeV nearest)
                (Graphs.nodeW nearest)
              -- 3. steer from nearest toward sample using dynamics
           in case Dynamics.steer nearestState qrand eps robot obs ws gen2 of
                Nothing -> go g gen2 (iter + 1) steerSuccesses -- steering failed
                Just ((states, ctrl, duration), gen3) -> do
                  -- 4. trajectory is already collision-checked in steer
                  -- 5. add new node with final state from trajectory
                  let finalState = last states
                      (a, gamma) = ctrl
                      (g', newId) = Graphs.addNode
                        (Dynamics.stateX finalState)
                        (Dynamics.stateY finalState)
                        (Dynamics.stateTheta finalState)
                        (Dynamics.stateV finalState)
                        (Dynamics.stateW finalState)
                        (Just (Graphs.nodeId nearest))
                        (Just ctrl)
                        (Just duration)
                        g
                   in -- 6. check goal (position must be in goal region AND robot must fit in workspace)
                      if inGoal (Dynamics.stateX finalState, Dynamics.stateY finalState) goal
                         && robotFitsInWorkspace (Dynamics.stateX finalState) (Dynamics.stateY finalState)
                        then do
                          putStrLn $ "\n\nGoal reached at iteration " ++ show iter ++ "!"
                          let path = Graphs.pathToRoot newId g'
                              -- Extract states from path for validation
                              pathStates = map nodeToState5D path
                          putStr "Validating path with full robot geometry... "
                          hFlush stdout
                          case Dynamics.validateFinalPathWithDiagnostics pathStates robot obs ws of
                            Nothing -> do
                              putStrLn "✓ Path valid!"
                              return (g', Right path)
                            Just failures -> do
                              putStrLn $ "✗ Validation failed (" ++ show (length failures) ++ " violations)"
                              -- Print first few failures for diagnostics
                              let firstFailures = take 3 failures
                              mapM_ printFailure firstFailures
                              when (length failures > 3) $
                                putStrLn $ "  ... and " ++ show (length failures - 3) ++ " more violations"
                              go g' gen3 (iter + 1) (steerSuccesses + 1) -- path validation failed
                        else go g' gen3 (iter + 1) (steerSuccesses + 1)

    -- Sample random 5D state in workspace
    sampleRandomState gen0 =
      let (x, gen1) = randomR (workspaceXMin ws, workspaceXMax ws) gen0
          (y, gen2) = randomR (workspaceYMin ws, workspaceYMax ws) gen1
          (theta, gen3) = randomR (0.0, 2 * pi) gen2
          (v, gen4) = randomR (-5.0, 5.0) gen3
          (w, gen5) = randomR (-pi/2, pi/2) gen4
       in (Dynamics.State5D x y theta v w, gen5)

    -- Sample state in goal region (with random velocities)
    -- Uses rejection sampling to ensure robot would fit in workspace
    sampleGoalState gen0 =
      let robotRadius = Dynamics.robotBoundingRadius robot
          -- Feasible workspace bounds (shrunk by robot bounding radius)
          xMin = workspaceXMin ws + robotRadius
          xMax = workspaceXMax ws - robotRadius
          yMin = workspaceYMin ws + robotRadius
          yMax = workspaceYMax ws - robotRadius
          -- Try to sample a feasible goal state (max 20 attempts)
          trySampleGoal gen n
            | n >= 20 = sampleRandomState gen  -- Fallback to random sampling
            | otherwise =
                let (angle, gen1) = randomR (0.0, 2 * pi) gen
                    (radius, gen2) = randomR (0.0, goalRadius goal) gen1
                    x = goalX goal + radius * cos angle
                    y = goalY goal + radius * sin angle
                 in if x >= xMin && x <= xMax && y >= yMin && y <= yMax
                      then
                        let (theta, gen3) = randomR (0.0, 2 * pi) gen2
                            (v, gen4) = randomR (-5.0, 5.0) gen3
                            (w, gen5) = randomR (-pi/2, pi/2) gen4
                         in (Dynamics.State5D x y theta v w, gen5)
                      else trySampleGoal gen2 (n + 1)
       in trySampleGoal gen0 0

    -- Check if robot at position (x, y) would fit entirely within workspace bounds
    -- Uses conservative bounding radius check (orientation-independent)
    robotFitsInWorkspace x y =
      let robotRadius = Dynamics.robotBoundingRadius robot
       in x - robotRadius >= workspaceXMin ws
            && x + robotRadius <= workspaceXMax ws
            && y - robotRadius >= workspaceYMin ws
            && y + robotRadius <= workspaceYMax ws

    -- Print validation failure for diagnostics
    printFailure :: Dynamics.ValidationFailure -> IO ()
    printFailure (Dynamics.WorkspaceBoundsViolation idx state pt violation) =
      putStrLn $ "  State #" ++ show idx ++ " at ("
                ++ show (Dynamics.stateX state) ++ ", "
                ++ show (Dynamics.stateY state) ++ ", θ="
                ++ show (Dynamics.stateTheta state) ++ "): "
                ++ "robot point " ++ show pt ++ " - " ++ violation
    printFailure (Dynamics.ObstacleCollision idx state pt obstacle) =
      putStrLn $ "  State #" ++ show idx ++ " at ("
                ++ show (Dynamics.stateX state) ++ ", "
                ++ show (Dynamics.stateY state) ++ ", θ="
                ++ show (Dynamics.stateTheta state) ++ "): "
                ++ "robot point " ++ show pt ++ " collides with obstacle at ("
                ++ show (Types.obstacleX obstacle) ++ ", "
                ++ show (Types.obstacleY obstacle) ++ ", r="
                ++ show (Types.obstacleRadius obstacle) ++ ")"

    -- Convert Node to State5D
    nodeToState5D n = Dynamics.State5D
      (Graphs.nodeX n)
      (Graphs.nodeY n)
      (Graphs.nodeTheta n)
      (Graphs.nodeV n)
      (Graphs.nodeW n)
