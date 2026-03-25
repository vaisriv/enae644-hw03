module Types
  ( -- Basic types
    Obstacle (..),
    Workspace (..),
    Goal (..),
    Motion (..),
  )
where

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
