module Graphs
  ( -- Types
    Node (..),
    Edge (..),
    Graph (..),
    -- Construction
    emptyGraph,
    addNode,
    addEdge,
    -- Queries
    getNode,
    getChildren,
    getParent,
    pathToRoot,
    -- Debug
    printGraph,
  )
where

import qualified Data.Map.Strict as Map

-------------------------------------------------------------------------------
-- Types
-------------------------------------------------------------------------------

-- | a node in the RRT tree
--   `nodeId`       : unique identifier (root = 0)
--   `nodeX`        : x coordinate in workspace R^2
--   `nodeY`        : y coordinate in workspace R^2
--   `nodeTheta`    : orientation in [0, 2π]
--   `nodeV`        : linear velocity in [-5, 5]
--   `nodeW`        : angular velocity in [-π/2, π/2]
--   `nodeParent`   : Just parentId, or Nothing for the root
--   `nodeControl`  : control (a, γ) that led to this node from parent
--   `nodeDuration` : duration of trajectory segment from parent
data Node = Node
  { nodeId :: Int,
    nodeX :: Double,
    nodeY :: Double,
    nodeTheta :: Double,
    nodeV :: Double,
    nodeW :: Double,
    nodeParent :: Maybe Int,
    nodeControl :: Maybe (Double, Double),
    nodeDuration :: Maybe Double
  }
  deriving (Show, Eq)

-- | a directed edge from parent to child, stored for convenience
--   the canonical parent reference lives on the Node itself
--   edges here allow efficient child-lookup
data Edge = Edge
  { edgeFrom :: Int,
    edgeTo :: Int
  }
  deriving (Show, Eq)

-- | the RRT search tree
--   `graphNodes`  : map from nodeId -> Node
--   `graphEdges`  : map from parentId -> [childId]
--   `graphNextId` : next available node id
data Graph = Graph
  { graphNodes :: Map.Map Int Node,
    graphEdges :: Map.Map Int [Int],
    graphNextId :: Int
  }
  deriving (Show)

-------------------------------------------------------------------------------
-- Construction
-------------------------------------------------------------------------------

-- | an empty graph with no nodes or edges
emptyGraph :: Graph
emptyGraph =
  Graph
    { graphNodes = Map.empty,
      graphEdges = Map.empty,
      graphNextId = 0
    }

-- | add a node to the graph, returning the updated graph and the new node's id
--   the caller supplies 5D state (x, y, theta, v, w), optional parent id,
--   optional control, and optional duration
--   the node id is assigned automatically from `graphNextId`
addNode ::
  Double ->
  Double ->
  Double ->
  Double ->
  Double ->
  Maybe Int ->
  Maybe (Double, Double) ->
  Maybe Double ->
  Graph ->
  (Graph, Int)
addNode x y theta v w parent control duration g =
  let nid = graphNextId g
      node =
        Node
          { nodeId = nid,
            nodeX = x,
            nodeY = y,
            nodeTheta = theta,
            nodeV = v,
            nodeW = w,
            nodeParent = parent,
            nodeControl = control,
            nodeDuration = duration
          }
      nodes' = Map.insert nid node (graphNodes g)
      -- register as child of parent
      edges' = case parent of
        Nothing -> graphEdges g
        Just pid -> Map.insertWith (++) pid [nid] (graphEdges g)
      g' =
        g
          { graphNodes = nodes',
            graphEdges = edges',
            graphNextId = nid + 1
          }
   in (g', nid)

-- | add an explicit edge (parent -> child)
--   only needed to record an edge that was not captured at node-insertion time
addEdge :: Int -> Int -> Graph -> Graph
addEdge from to g =
  g {graphEdges = Map.insertWith (++) from [to] (graphEdges g)}

-------------------------------------------------------------------------------
-- Queries
-------------------------------------------------------------------------------

-- | look up a node by id
--   returns Nothing if not found
getNode :: Int -> Graph -> Maybe Node
getNode nid g = Map.lookup nid (graphNodes g)

-- | get the ids of all direct children of a node
getChildren :: Int -> Graph -> [Int]
getChildren nid g = Map.findWithDefault [] nid (graphEdges g)

-- | get the parent node of a given node, if it exists
getParent :: Int -> Graph -> Maybe Node
getParent nid g = do
  node <- getNode nid g
  pid <- nodeParent node
  getNode pid g

-- | trace the path from a node back to the root, returning nodes in
--   root-to-node order
pathToRoot :: Int -> Graph -> [Node]
pathToRoot nid g = reverse $ go nid
  where
    go i = case getNode i g of
      Nothing -> []
      Just node ->
        node : case nodeParent node of
          Nothing -> []
          Just pid -> go pid

-------------------------------------------------------------------------------
-- Debug
-------------------------------------------------------------------------------

-- | print the graph to stdout for debugging
printGraph :: Graph -> IO ()
printGraph g = do
  putStrLn $ "Graph: " ++ show (Map.size (graphNodes g)) ++ " nodes"
  mapM_ printNode (Map.elems (graphNodes g))
  where
    printNode n =
      putStrLn $
        "  Node "
          ++ show (nodeId n)
          ++ " ("
          ++ show (nodeX n)
          ++ ", "
          ++ show (nodeY n)
          ++ ")"
          ++ " parent="
          ++ maybe "none" show (nodeParent n)
