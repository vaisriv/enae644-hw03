{-# LANGUAGE DeriveGeneric #-}

module RRT
  ( Workspace (..),
  -- Obstacle (..),
  -- Problem (..),
  -- rrt,
  )
where

import GHC.Generics
import qualified Graphs (Edge, Graph, Node)
import System.FilePath ((</>))

-- graph structures
data Workspace = Workspace
  { xBounds :: (Int, Int),
    yBounds :: (Int, Int)
  }
  deriving (Generic, Show)

-- data Obstacle = Obstacle
--   { xBounds :: (Int, Int),
--     yBounds :: (Int, Int)
--   } deriving (Generic, Show)
--
-- data Problem = Problem
--   { xBounds :: (Int, Int),
--     yBounds :: (Int, Int)
--   } deriving (Generic, Show)

-- rrt algorithm
-- rrt :: Graphs.Graph -> IO ()
-- rrt graph = do
--   putStrLn "Nodes"
