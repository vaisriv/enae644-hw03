{-# LANGUAGE DeriveGeneric #-}

module Graphs
  ( Graph (..),
    Node (..),
    Edge (..),
    readGraph,
    printGraph,
  )
where

import Data.Char (isSpace)
import GHC.Generics
import System.FilePath ((</>))

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
-- read from files, need `nodes.txt` and `edges.txt` (or `edges_with_costs.txt`) in provided path
readGraph :: FilePath -> Bool -> IO Graph
readGraph graphpath weighted = do
  nodesContent <- readFile (graphpath </> "nodes.txt")
  edgesContent <- readFile (graphpath </> edgesFile)
  let graph =
        Graph
          { nodes = parseNodes nodesContent,
            edges = parseEdges weighted edgesContent
          }
  return graph
  where
    edgesFile = if weighted then "edges_with_costs.txt" else "edges.txt"

-- node/edge parsing
parseRow :: String -> [String]
parseRow = map trim . splitOn ','
  where
    trim = dropWhile isSpace . reverse . dropWhile isSpace . reverse
    splitOn _ "" = [""]
    splitOn delim str =
      let (field, rest) = break (== delim) str
       in field : case rest of
            [] -> []
            (_ : xs) -> splitOn delim xs

parseNodes :: String -> [Node]
parseNodes content =
  map parseNode . tail . lines $ content
  where
    parseNode line =
      let [idStr, xStr, yStr] = parseRow line
       in Node
            { nodeID = read idStr,
              xCoord = read xStr,
              yCoord = read yStr
            }

parseEdges :: Bool -> String -> [Edge]
parseEdges weighted content =
  map parseEdge . tail . lines $ content
  where
    parseEdge line
      | weighted =
          let [startStr, endStr, wStr] = parseRow line
           in Edge
                { startNode = read startStr,
                  endNode = read endStr,
                  weight = Just (read wStr)
                }
      | otherwise =
          let [startStr, endStr] = parseRow line
           in Edge
                { startNode = read startStr,
                  endNode = read endStr,
                  weight = Nothing
                }

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
