module Graphs
  ( Graph (..),
    Node (..),
    Edge (..),
    AStarResult (..),
    readGraph,
    printGraph,
    astar,
  )
where

import Data.Char (isSpace)
import Data.Heap (Entry (..))
import qualified Data.Heap as Heap
import Data.Map.Strict (Map)
import qualified Data.Map.Strict as Map
import Data.Maybe (fromMaybe)
import System.FilePath ((</>))

-- graph structures
data Node = Node
  { nodeID :: Int,
    xCoord :: Float,
    yCoord :: Float
  }
  deriving (Show)

data Edge = Edge
  { startNode :: Int,
    endNode :: Int,
    weight :: Maybe Float
  }
  deriving (Show)

data Graph = Graph
  { nodes :: [Node],
    edges :: [Edge]
  }
  deriving (Show)

-- result of a successful A* search
--   'pathNodes' – nodes from goal (top) to start (bottom), goal-first order
--   'searchTreeEdges' – every (child, parent) pair that A* ever relaxed
data AStarResult = AStarResult
  { pathNodes :: [Node], -- goal first, start last
    searchTreeEdges :: [(Node, Node)] -- (child, parent) pairs
  }

-- graph IO
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

-- the A* algorithm
heuristic :: Node -> Node -> Float
heuristic a b =
  let dx = xCoord a - xCoord b
      dy = yCoord a - yCoord b
   in sqrt (dx * dx + dy * dy)

buildNodeMap :: Graph -> Map Int Node
buildNodeMap g = Map.fromList [(nodeID n, n) | n <- nodes g]

buildAdjacency :: Graph -> Map Int [(Int, Float)]
buildAdjacency g = foldr addEdge Map.empty (edges g)
  where
    addEdge e acc =
      let w = fromMaybe 1.0 (weight e)
          src = startNode e
          dst = endNode e
       in Map.insertWith (++) src [(dst, w)] acc

-- run A* on the graph [startID->goalID]
-- NOTE: always returns the search tree
-- path is Just [nodes] on success, Nothing if no path exists
astar :: Graph -> Int -> Int -> (Maybe AStarResult, [(Node, Node)])
astar g startID goalID = go initOpen initCostSoFar initCameFrom []
  where
    nodeMap = buildNodeMap g
    adjacency = buildAdjacency g

    lookupNode nid =
      case Map.lookup nid nodeMap of
        Just n -> n
        Nothing -> error $ "Node " ++ show nid ++ " not found in graph"

    goalNode = lookupNode goalID
    startNode = lookupNode startID

    initOpen = Heap.singleton (Entry (heuristic startNode goalNode) startID)
    initCostSoFar = Map.singleton startID 0.0
    initCameFrom = Map.empty :: Map Int Int

    go open costSoFar cameFrom treeAcc
      | Heap.null open = (Nothing, treeAcc)
      | otherwise =
          let Entry _ current = Heap.minimum open
              open' = Heap.deleteMin open
           in if current == goalID
                then
                  ( Just $
                      AStarResult
                        { pathNodes = reconstructPath cameFrom goalID,
                          searchTreeEdges = treeAcc
                        },
                    treeAcc
                  )
                else
                  let neighbours = fromMaybe [] (Map.lookup current adjacency)
                      (open'', costSoFar', cameFrom', treeAcc') =
                        foldr (relax current) (open', costSoFar, cameFrom, treeAcc) neighbours
                   in go open'' costSoFar' cameFrom' treeAcc'

    relax current (nbr, w) (openAcc, costAcc, cameAcc, treeAcc) =
      let tentative = (costAcc Map.! current) + w
       in case Map.lookup nbr costAcc of
            Just existing | existing <= tentative -> (openAcc, costAcc, cameAcc, treeAcc)
            _ ->
              let h = heuristic (lookupNode nbr) goalNode
                  f = tentative + h
               in ( Heap.insert (Entry f nbr) openAcc,
                    Map.insert nbr tentative costAcc,
                    Map.insert nbr current cameAcc,
                    (lookupNode nbr, lookupNode current) : treeAcc
                  )

    -- reconstructs path with goal first, start last
    reconstructPath cameFrom nid =
      let node = lookupNode nid
       in case Map.lookup nid cameFrom of
            Nothing -> [node]
            Just parent -> node : reconstructPath cameFrom parent
