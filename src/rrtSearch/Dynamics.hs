module Dynamics
  ( -- Types
    State5D (State5D, stateX, stateY, stateTheta, stateV, stateW),
    Control,
    Trajectory,
    RobotShape (RobotShape, shapePoints, shapeHull),
    -- Dynamics
    carDynamics,
    integrateRK4,
    integrateTrajectory,
    subsampleTrajectory,
    -- Geometry
    convexHull,
    transformPoint,
    angleDiff,
    distState5D,
    robotBoundingRadius,
    -- Collision
    pointClearOfCircle,
    stateCollisionFree,
    trajectoryCollisionFree,
    validateFinalPath,
    -- Steering (2P-BVP)
    steer,
  )
where

import Data.List (minimum, sortBy)
import qualified System.Random
import qualified Types (Obstacle (..), Workspace (..))

-------------------------------------------------------------------------------
-- Types
-------------------------------------------------------------------------------

-- | 5D configuration space state: (x, y, theta, v, w)
data State5D = State5D
  { stateX :: Double, -- x position
    stateY :: Double, -- y position
    stateTheta :: Double, -- orientation [0, 2π]
    stateV :: Double, -- linear velocity [-5, 5]
    stateW :: Double -- angular velocity [-π/2, π/2]
  }
  deriving (Show, Eq)

-- | control inputs: (a, gamma) where a is acceleration, gamma is steering acceleration
type Control = (Double, Double)

-- | a trajectory is a sequence of states with time stamps
--   includes the control and duration that generated it
type Trajectory = ([State5D], Control, Double)

-- | robot shape: all points and convex hull
data RobotShape = RobotShape
  { shapePoints :: [(Double, Double)], -- all 37 points
    shapeHull :: [(Double, Double)] -- convex hull (~8 points)
  }
  deriving (Show)

-------------------------------------------------------------------------------
-- Car Dynamics
-------------------------------------------------------------------------------

-- | car dynamics: dx/dt = f(state, control)
--   returns state derivative [dx/dt, dy/dt, dtheta/dt, dv/dt, dw/dt]
carDynamics :: State5D -> Control -> (Double, Double, Double, Double, Double)
carDynamics (State5D x y theta v w) (a, gamma) =
  ( v * cos theta, -- dx/dt
    v * sin theta, -- dy/dt
    w, -- dtheta/dt
    a, -- dv/dt
    gamma -- dw/dt
  )

-- | single RK4 integration step
--   advances state by time dt using control input
integrateRK4 :: Double -> State5D -> Control -> State5D
integrateRK4 dt state ctrl =
  let -- k1 = f(t, y)
      (dx1, dy1, dtheta1, dv1, dw1) = carDynamics state ctrl
      k1 = State5D dx1 dy1 dtheta1 dv1 dw1

      -- k2 = f(t + dt/2, y + dt/2 * k1)
      state2 =
        State5D
          (stateX state + dt / 2 * dx1)
          (stateY state + dt / 2 * dy1)
          (stateTheta state + dt / 2 * dtheta1)
          (stateV state + dt / 2 * dv1)
          (stateW state + dt / 2 * dw1)
      (dx2, dy2, dtheta2, dv2, dw2) = carDynamics state2 ctrl
      k2 = State5D dx2 dy2 dtheta2 dv2 dw2

      -- k3 = f(t + dt/2, y + dt/2 * k2)
      state3 =
        State5D
          (stateX state + dt / 2 * dx2)
          (stateY state + dt / 2 * dy2)
          (stateTheta state + dt / 2 * dtheta2)
          (stateV state + dt / 2 * dv2)
          (stateW state + dt / 2 * dw2)
      (dx3, dy3, dtheta3, dv3, dw3) = carDynamics state3 ctrl
      k3 = State5D dx3 dy3 dtheta3 dv3 dw3

      -- k4 = f(t + dt, y + dt * k3)
      state4 =
        State5D
          (stateX state + dt * dx3)
          (stateY state + dt * dy3)
          (stateTheta state + dt * dtheta3)
          (stateV state + dt * dv3)
          (stateW state + dt * dw3)
      (dx4, dy4, dtheta4, dv4, dw4) = carDynamics state4 ctrl
      k4 = State5D dx4 dy4 dtheta4 dv4 dw4

      -- y_next = y + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
   in State5D
        (stateX state + dt / 6 * (dx1 + 2 * dx2 + 2 * dx3 + dx4))
        (stateY state + dt / 6 * (dy1 + 2 * dy2 + 2 * dy3 + dy4))
        (stateTheta state + dt / 6 * (dtheta1 + 2 * dtheta2 + 2 * dtheta3 + dtheta4))
        (stateV state + dt / 6 * (dv1 + 2 * dv2 + 2 * dv3 + dv4))
        (stateW state + dt / 6 * (dw1 + 2 * dw2 + 2 * dw3 + dw4))

-- | integrate trajectory from initial state for given duration
--   returns list of states at dt intervals
integrateTrajectory :: Double -> State5D -> Control -> Double -> [State5D]
integrateTrajectory dt initState ctrl duration =
  let numSteps = ceiling (duration / dt)
   in go numSteps initState []
  where
    go 0 state acc = reverse (state : acc)
    go n state acc =
      let newState = integrateRK4 dt state ctrl
       in go (n - 1) newState (state : acc)

-- | subsample trajectory to ensure consecutive states are <= delta apart in (x, y)
subsampleTrajectory :: Double -> [State5D] -> [State5D]
subsampleTrajectory delta states = case states of
  [] -> []
  [s] -> [s]
  (s1 : rest) -> s1 : go s1 rest
  where
    go _ [] = []
    go prev (s : ss) =
      let d = sqrt ((stateX s - stateX prev) ^ (2 :: Int) + (stateY s - stateY prev) ^ (2 :: Int))
       in if d <= delta
            then s : go s ss
            else
              -- interpolate additional points between prev and s
              let numPoints = ceiling (d / delta)
                  interpStates = [interpolateStates prev s (fromIntegral i / fromIntegral numPoints) | i <- [1 .. numPoints - 1]]
               in interpStates ++ (s : go s ss)

    interpolateStates s1 s2 t =
      State5D
        (stateX s1 + t * (stateX s2 - stateX s1))
        (stateY s1 + t * (stateY s2 - stateY s1))
        (stateTheta s1 + t * (stateTheta s2 - stateTheta s1))
        (stateV s1 + t * (stateV s2 - stateV s1))
        (stateW s1 + t * (stateW s2 - stateW s1))

-------------------------------------------------------------------------------
-- Geometry
-------------------------------------------------------------------------------

-- | compute angle difference handling wraparound
angleDiff :: Double -> Double -> Double
angleDiff a1 a2 =
  let d = abs (a2 - a1)
   in min d (2 * pi - d)

-- | weighted distance metric for 5D configuration space
distState5D :: State5D -> State5D -> Double
distState5D s1 s2 =
  sqrt
    ( 1.0 * ((stateX s2 - stateX s1) ^ (2 :: Int) + (stateY s2 - stateY s1) ^ (2 :: Int))
        + 0.3 * angleDiff (stateTheta s1) (stateTheta s2) ^ (2 :: Int)
        + 0.2 * (stateV s2 - stateV s1) ^ (2 :: Int)
        + 0.1 * (stateW s2 - stateW s1) ^ (2 :: Int)
    )

-- | compute the bounding radius of a robot shape
--   returns the maximum distance from origin to any point in the shape
robotBoundingRadius :: RobotShape -> Double
robotBoundingRadius robot =
  maximum $ map (\(x, y) -> sqrt (x * x + y * y)) (shapePoints robot)

-- | transform robot point from body frame to world frame
transformPoint :: Double -> Double -> Double -> (Double, Double) -> (Double, Double)
transformPoint x y theta (px, py) =
  ( x + px * cos theta - py * sin theta,
    y + px * sin theta + py * cos theta
  )

-- | compute convex hull using Graham scan
convexHull :: [(Double, Double)] -> [(Double, Double)]
convexHull points
  | length points < 3 = points
  | otherwise =
      let -- find point with lowest y-coordinate (leftmost if tie)
          p0 = minimum points
          -- sort points by polar angle with respect to p0
          sorted = sortByAngle p0 (filter (/= p0) points)
          -- build convex hull using Graham scan
       in grahamScan [p0] sorted
  where
    sortByAngle p0 pts =
      let angleFrom (x1, y1) (x2, y2) = atan2 (y2 - y1) (x2 - x1)
       in sortBy (\p1 p2 -> compare (angleFrom p0 p1) (angleFrom p0 p2)) pts

    grahamScan hull [] = hull
    grahamScan hull (p : ps) = case hull of
      [] -> grahamScan [p] ps
      [h1] -> grahamScan [p, h1] ps
      (h1 : h2 : rest) ->
        if ccw h2 h1 p
          then grahamScan (p : hull) ps
          else grahamScan (h2 : rest) (p : ps)

    -- counter-clockwise test
    ccw (x1, y1) (x2, y2) (x3, y3) =
      (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1) > 0

-------------------------------------------------------------------------------
-- Collision Checking
-------------------------------------------------------------------------------

-- | check if point is within workspace bounds
pointInWorkspace :: (Double, Double) -> Types.Workspace -> Bool
pointInWorkspace (x, y) ws =
  x >= Types.workspaceXMin ws
    && x <= Types.workspaceXMax ws
    && y >= Types.workspaceYMin ws
    && y <= Types.workspaceYMax ws

-- | check if point is clear of circular obstacle
pointClearOfCircle :: (Double, Double) -> Types.Obstacle -> Bool
pointClearOfCircle (x, y) obs =
  let dx = x - Types.obstacleX obs
      dy = y - Types.obstacleY obs
      distSq = dx * dx + dy * dy
   in distSq > Types.obstacleRadius obs ^ (2 :: Int)

-- | check if state is collision-free using convex hull (fast)
stateCollisionFree :: State5D -> RobotShape -> [Types.Obstacle] -> Types.Workspace -> Bool
stateCollisionFree state robot obstacles workspace =
  let transformedHull = map (transformPoint (stateX state) (stateY state) (stateTheta state)) (shapeHull robot)
   in all (\pt -> pointInWorkspace pt workspace && all (pointClearOfCircle pt) obstacles) transformedHull

-- | check if state is collision-free using all robot points (thorough)
validateState :: State5D -> RobotShape -> [Types.Obstacle] -> Types.Workspace -> Bool
validateState state robot obstacles workspace =
  let transformedAll = map (transformPoint (stateX state) (stateY state) (stateTheta state)) (shapePoints robot)
   in all (\pt -> pointInWorkspace pt workspace && all (pointClearOfCircle pt) obstacles) transformedAll

-- | check if trajectory is collision-free using convex hull
trajectoryCollisionFree :: [State5D] -> RobotShape -> [Types.Obstacle] -> Types.Workspace -> Bool
trajectoryCollisionFree states robot obstacles workspace =
  let subsampled = subsampleTrajectory 0.5 states
   in all (\s -> stateCollisionFree s robot obstacles workspace) subsampled

-- | validate final path using all robot points
validateFinalPath :: [State5D] -> RobotShape -> [Types.Obstacle] -> Types.Workspace -> Bool
validateFinalPath states robot obstacles workspace =
  let subsampled = subsampleTrajectory 0.5 states
   in all (\s -> validateState s robot obstacles workspace) subsampled

-------------------------------------------------------------------------------
-- Steering Function (2P-BVP Solver)
-------------------------------------------------------------------------------

-- | steer from start state to goal state
--   returns trajectory if successful, Nothing otherwise
--   uses random shooting
steer :: System.Random.RandomGen g => State5D -> State5D -> Double -> RobotShape -> [Types.Obstacle] -> Types.Workspace -> g -> Maybe (Trajectory, g)
steer startState goalState epsilon robot obstacles workspace gen =
  let (bestTraj, gen') = tryRandomShooting numAttempts gen Nothing
   in case bestTraj of
        Nothing -> Nothing
        Just traj -> Just (traj, gen')
  where
    -- Parameters
    numAttempts = 20
    dt = 0.01
    maxDuration = 5.0
    minDuration = 0.1
    aMax = 2.0
    gammaMax = pi / 2

    tryRandomShooting 0 g best = (best, g)
    tryRandomShooting n g best =
      let -- Sample random control and duration
          (a, g1) = System.Random.randomR (-aMax, aMax) g
          (gamma, g2) = System.Random.randomR (-gammaMax, gammaMax) g1
          (duration, g3) = System.Random.randomR (minDuration, maxDuration) g2
          control = (a, gamma)

          -- Integrate trajectory
          states = integrateTrajectory dt startState control duration

          -- Check if trajectory is collision-free
          collisionFree = trajectoryCollisionFree states robot obstacles workspace

          -- Evaluate distance to goal
          finalState = last states
          dist = distState5D finalState goalState

          -- Update best if this is better and collision-free
          newBest = if collisionFree && dist < epsilon
                    then case best of
                           Nothing -> Just (states, control, duration)
                           Just (_, _, _) ->
                             if dist < epsilon / 2
                             then Just (states, control, duration)
                             else best
                    else best
       in tryRandomShooting (n - 1) g3 newBest
