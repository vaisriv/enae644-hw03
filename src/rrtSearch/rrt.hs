module RRT
  ( -- Types
    Obstacle (..),
    Workspace (..),
    Goal (..),
    Motion (..),
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

import qualified Data.List (minimumBy)
import qualified Data.Map.Strict as Map
import Data.Ord (comparing)
import qualified Graphs
  ( Graph (..),
    Node (..),
    addNode,
    emptyGraph,
    graphNodes,
    pathToRoot,
  )
import System.Random (RandomGen, randomR)

-------------------------------------------------------------------------------
-- Types
-------------------------------------------------------------------------------

-- | a (circular) disc obstacle in R^2
data Obstacle = Obstacle
  { obstacleX :: Double,
    obstacleY :: Double,
    obstacleRadius :: Double
  }
  deriving (Show)

-- | axis-aligned rectangular workspace bounds
data Workspace = Workspace
  { workspaceXMin :: Double,
    workspaceXMax :: Double,
    workspaceYMin :: Double,
    workspaceYMax :: Double
  }
  deriving (Show)

-- | circular goal region
data Goal = Goal
  { goalX :: Double,
    goalY :: Double,
    goalRadius :: Double
  }
  deriving (Show)

-- | motion parameters
--   epsilon is max extension distance per step
data Motion = Motion
  { motionEpsilon :: Double
  }
  deriving (Show)

-- | a fully-specified RRT problem instance
data Problem = Problem
  { problemWorkspace :: Workspace,
    problemStart :: (Double, Double),
    problemGoal :: Goal,
    problemMotion :: Motion,
    problemObstacles :: [Obstacle]
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

-- | find the nearest node in the graph to a query point
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

-- | run RRT
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
  (Graphs.Graph, Either String [Graphs.Node])
rrt problem maxIter gen =
  let (sx, sy) = problemStart problem
      (initGraph, _) = Graphs.addNode sx sy Nothing Graphs.emptyGraph
   in go initGraph gen 0
  where
    ws = problemWorkspace problem
    goal = problemGoal problem
    obs = problemObstacles problem
    eps = motionEpsilon (problemMotion problem)

    go g gen' iter
      | iter >= maxIter =
          ( g,
            Left $ "No path found after " ++ show maxIter ++ " iterations"
          )
      | otherwise =
          -- 1. sample a random point in the workspace
          let (rx, gen1) = randomR (workspaceXMin ws, workspaceXMax ws) gen'
              (ry, gen2) = randomR (workspaceYMin ws, workspaceYMax ws) gen1
              qrand = (rx, ry)
              -- 2. find nearest node in tree
              nearest = nearestNode qrand g
              npos = (Graphs.nodeX nearest, Graphs.nodeY nearest)
              -- 3. extend toward sample
              qnew = extendToward npos qrand eps
              -- 4. collision check
              clear = segmentClear npos qnew obs
           in if not clear
                then go g gen2 (iter + 1)
                else
                  -- 5. add new node
                  let (g', newId) = Graphs.addNode (fst qnew) (snd qnew) (Just (Graphs.nodeId nearest)) g
                   in -- 6. check goal
                      if inGoal qnew goal
                        then (g', Right (Graphs.pathToRoot newId g'))
                        else go g' gen2 (iter + 1)
