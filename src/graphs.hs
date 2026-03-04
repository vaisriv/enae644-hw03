{-# LANGUAGE DeriveGeneric #-}

module Graphs
  ( Graph (..),
    Node (..),
    Edge (..),
    printGraph,
  )
where

import GHC.Generics

-- graph structures
data Graph = Graph
  { nodes :: [Node],
    edges :: [Edge]
  }
  deriving (Generic, Show)

data Node = Node
  { nodeID :: Int,
    xCoord :: Float,
    yCoord :: Float
  }
  deriving (Generic, Show)

data Edge = Edge
  { startNode :: Int,
    endNode :: Int,
    weight :: Maybe Float
  }
  deriving (Generic, Show)

-- graph IO
-- print graph to stdout
printGraph :: Graph -> IO ()
printGraph graph = do
  putStrLn "Nodes:"
  mapM_ printNode (nodes graph)
  putStrLn "Edges:"
  mapM_ printEdge (edges graph)
  where
    printNode n =
      putStrLn $
        "  Node "
          ++ show (nodeID n)
          ++ " at ("
          ++ show (xCoord n)
          ++ ", "
          ++ show (yCoord n)
          ++ ")"
    printEdge e =
      putStrLn $
        "  Edge "
          ++ show (startNode e)
          ++ " -> "
          ++ show (endNode e)
          ++ weightStr (weight e)
    weightStr Nothing = ""
    weightStr (Just w) = " (weight: " ++ show w ++ ")"
