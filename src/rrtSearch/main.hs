{-# LANGUAGE OverloadedStrings #-}

module Main where

import Data.Aeson
  ( FromJSON (..),
    eitherDecodeFileStrict,
    withObject,
    (.:),
  )
import qualified Data.Map.Strict as Map
import qualified Dynamics
  ( RobotShape (..),
    convexHull,
  )
import qualified Graphs
  ( Graph (..),
    Node (..),
    emptyGraph,
    graphNodes,
    printGraph,
  )
import qualified RRT
  ( Goal (..),
    Motion (..),
    Obstacle (..),
    Problem (..),
    Workspace (..),
    rrt,
  )
import qualified System.Directory (createDirectoryIfMissing, doesFileExist)
import qualified System.Environment
import qualified System.Exit
import qualified System.FilePath (FilePath, (</>))
import System.Random (initStdGen)

-------------------------------------------------------------------------------
-- Args
-------------------------------------------------------------------------------

data Args = Args
  { inputPath :: System.FilePath.FilePath,
    outputPath :: System.FilePath.FilePath,
    problemNum :: String,
    maxAttempts :: Int
  }

defaultArgs :: Args
defaultArgs =
  Args
    { inputPath = "./data",
      outputPath = "./outputs",
      problemNum = "01",
      maxAttempts = 100000
    }

-------------------------------------------------------------------------------
-- JSON types (intermediate representations for aeson parsing)
-------------------------------------------------------------------------------

-- | intermediate type for deserializing the workspace object
--     { "x": [xmin, xmax], "y": [ymin, ymax] }
data JsonWorkspace = JsonWorkspace
  { jwX :: [Double],
    jwY :: [Double]
  }

instance FromJSON JsonWorkspace where
  parseJSON = withObject "Workspace" $ \o ->
    JsonWorkspace <$> o .: "x" <*> o .: "y"

-- | intermediate type for deserializing the start/point objects
--     { "x": ..., "y": ..., "theta_pi_rad": ... }
data JsonPoint = JsonPoint
  { jpX :: Double,
    jpY :: Double,
    jpTheta :: Double
  }

instance FromJSON JsonPoint where
  parseJSON = withObject "Point" $ \o ->
    JsonPoint <$> o .: "x" <*> o .: "y" <*> o .: "theta_pi_rad"

-- | intermediate type for deserializing the goal object
--     { "x": ..., "y": ..., "radius": ... }
data JsonGoal = JsonGoal
  { jgX :: Double,
    jgY :: Double,
    jgRadius :: Double
  }

instance FromJSON JsonGoal where
  parseJSON = withObject "Goal" $ \o ->
    JsonGoal <$> o .: "x" <*> o .: "y" <*> o .: "radius"

-- | intermediate type for deserializing the motion object
--     { "epsilon": ... }
data JsonMotion = JsonMotion
  { jmEpsilon :: Double
  }

instance FromJSON JsonMotion where
  parseJSON = withObject "Motion" $ \o ->
    JsonMotion <$> o .: "epsilon"

-- | top-level problem file
data JsonProblem = JsonProblem
  { jpWorkspace :: JsonWorkspace,
    jpStart :: JsonPoint,
    jpGoal :: JsonGoal,
    jpMotion :: JsonMotion
  }

instance FromJSON JsonProblem where
  parseJSON = withObject "Problem" $ \o ->
    JsonProblem
      <$> o .: "workspace"
      <*> o .: "start"
      <*> o .: "goal"
      <*> o .: "motion"

-------------------------------------------------------------------------------
-- Parsing
-------------------------------------------------------------------------------

-- | parse `obstacles.txt` into a list of Obstacles
--
--   format:
--     num_obstacles
--     x_1, y_1, radius_1
--     ...
parseObstacles :: System.FilePath.FilePath -> IO [RRT.Obstacle]
parseObstacles path = do
  contents <- readFile path
  let ls = lines contents
  case ls of
    [] -> fail "obstacles.txt is empty"
    (_ : rest) -> mapM parseLine (filter (not . null) rest)
  where
    parseLine l = case map (read . trim) (splitOn ',' l) of
      [x, y, r] -> return $ RRT.Obstacle x y r
      _ -> fail $ "malformed obstacle line: " ++ l
    trim = reverse . dropWhile (== ' ') . reverse . dropWhile (== ' ')
    splitOn _ [] = [""]
    splitOn delim (c : cs)
      | c == delim = "" : splitOn delim cs
      | otherwise = let (w : ws) = splitOn delim cs in (c : w) : ws

-- | parse `robot.txt` into a RobotShape with convex hull
--
--   format:
--     x_1, y_1
--     x_2, y_2
--     ...
parseRobot :: System.FilePath.FilePath -> IO Dynamics.RobotShape
parseRobot path = do
  contents <- readFile path
  let ls = lines contents
  points <- mapM parseLine (filter (not . null) ls)
  let hull = Dynamics.convexHull points
  return $ Dynamics.RobotShape {Dynamics.shapePoints = points, Dynamics.shapeHull = hull}
  where
    parseLine l = case map (read . trim) (splitOn ',' l) of
      [x, y] -> return (x, y)
      _ -> fail $ "malformed robot line: " ++ l
    trim = reverse . dropWhile (== ' ') . reverse . dropWhile (== ' ')
    splitOn _ [] = [""]
    splitOn delim (c : cs)
      | c == delim = "" : splitOn delim cs
      | otherwise = let (w : ws) = splitOn delim cs in (c : w) : ws

-- | parse `pXX.json` into a Problem
--   obstacles and robot shape are passed in separately
parseProblem :: System.FilePath.FilePath -> [RRT.Obstacle] -> Dynamics.RobotShape -> IO RRT.Problem
parseProblem path obs robot = do
  result <- eitherDecodeFileStrict path
  case result of
    Left err -> fail $ "failed to parse problem file: " ++ err
    Right jp -> return $ toProblem jp
  where
    toProblem jp =
      let startPt = jpStart jp
          thetaRad = jpTheta startPt * pi -- convert from pi-radians to radians
       in RRT.Problem
            { RRT.problemWorkspace = toWorkspace (jpWorkspace jp),
              RRT.problemStart = (jpX startPt, jpY startPt, thetaRad, 0.0, 0.0), -- (x, y, theta, v=0, w=0)
              RRT.problemGoal = toGoal (jpGoal jp),
              RRT.problemMotion = toMotion (jpMotion jp),
              RRT.problemObstacles = obs,
              RRT.problemRobot = robot
            }
    toWorkspace jw = case (jwX jw, jwY jw) of
      ([xmin, xmax], [ymin, ymax]) ->
        RRT.Workspace xmin xmax ymin ymax
      _ -> error "workspace x/y bounds must each be a 2-element array"
    toGoal jg =
      RRT.Goal
        { RRT.goalX = jgX jg,
          RRT.goalY = jgY jg,
          RRT.goalRadius = jgRadius jg
        }
    toMotion jm =
      RRT.Motion
        { RRT.motionEpsilon = jmEpsilon jm
        }

-------------------------------------------------------------------------------
-- Serialization
-------------------------------------------------------------------------------

-- | write `search_tree.csv` from the final RRT graph
--
--   format (with header):
--     node_id,x,y,theta,v,w,parent_id
--     ...
--   root node has parent_id = -1
writeSearchTree :: System.FilePath.FilePath -> Graphs.Graph -> IO ()
writeSearchTree path g = do
  let header = "node_id,x,y,theta,v,w,parent_id"
      rows = map nodeRow (Map.elems (Graphs.graphNodes g))
  writeFile path (unlines (header : rows))
  where
    nodeRow n =
      show (Graphs.nodeId n)
        ++ ","
        ++ show (Graphs.nodeX n)
        ++ ","
        ++ show (Graphs.nodeY n)
        ++ ","
        ++ show (Graphs.nodeTheta n)
        ++ ","
        ++ show (Graphs.nodeV n)
        ++ ","
        ++ show (Graphs.nodeW n)
        ++ ","
        ++ maybe "-1" show (Graphs.nodeParent n)

-- | write `path.csv` from the RRT result
--
--   success format (with header):
--     t,x,y,theta,v,w,a,gamma
--     ...
--   Each line represents a trajectory segment from time t_i to t_{i+1}
--   with the state at t_i and the controls (a, gamma) applied during that segment
--
--   failure format:
--     ERROR: <message>
writePath :: System.FilePath.FilePath -> Either String [Graphs.Node] -> IO ()
writePath path (Left errMsg) =
  writeFile path ("ERROR: " ++ errMsg ++ "\n")
writePath path (Right nodes) = do
  let header = "t,x,y,theta,v,w,a,gamma"
      rows = trajectoryRows nodes 0.0
  writeFile path (unlines (header : rows))
  where
    -- Generate trajectory rows with cumulative time
    trajectoryRows [] _ = []
    trajectoryRows [n] t =
      -- Final node: output state with zero controls
      [formatRow t n 0.0 0.0]
    trajectoryRows (n : rest@(next : _)) t =
      let -- Get control and duration from the next node (it stores how it got there from n)
          (a, gamma) = maybe (0.0, 0.0) id (Graphs.nodeControl next)
          duration = maybe 0.0 id (Graphs.nodeDuration next)
          row = formatRow t n a gamma
       in row : trajectoryRows rest (t + duration)

    formatRow t n a gamma =
      show t
        ++ ","
        ++ show (Graphs.nodeX n)
        ++ ","
        ++ show (Graphs.nodeY n)
        ++ ","
        ++ show (Graphs.nodeTheta n)
        ++ ","
        ++ show (Graphs.nodeV n)
        ++ ","
        ++ show (Graphs.nodeW n)
        ++ ","
        ++ show a
        ++ ","
        ++ show gamma

-------------------------------------------------------------------------------
-- Main
-------------------------------------------------------------------------------

main :: IO ()
main = do
  args <- System.Environment.getArgs >>= parse
  let probFile = inputPath args System.FilePath.</> ("p" ++ problemNum args ++ ".json")
      obsFile = inputPath args System.FilePath.</> "obstacles.txt"
      robotFile = inputPath args System.FilePath.</> "robot.txt"
      outDir = outputPath args System.FilePath.</> problemNum args
      treeFile = outDir System.FilePath.</> "search_tree.csv"
      pathFile = outDir System.FilePath.</> "path.csv"
  -- verify inputs exist
  probExists <- System.Directory.doesFileExist probFile
  obsExists <- System.Directory.doesFileExist obsFile
  robotExists <- System.Directory.doesFileExist robotFile
  if not probExists
    then putStrLn ("Error: problem file not found: " ++ probFile) >> die
    else return ()
  if not obsExists
    then putStrLn ("Error: obstacles file not found: " ++ obsFile) >> die
    else return ()
  if not robotExists
    then putStrLn ("Error: robot file not found: " ++ robotFile) >> die
    else return ()
  -- ensure output directory exists
  System.Directory.createDirectoryIfMissing True outDir
  -- parse inputs
  obstacles <- parseObstacles obsFile
  robot <- parseRobot robotFile
  problem <- parseProblem probFile obstacles robot
  -- run RRT
  gen <- initStdGen
  let (tree, result) = RRT.rrt problem (maxAttempts args) gen
  -- write outputs
  writeSearchTree treeFile tree
  writePath pathFile result
  -- report
  putStrLn $ "rrtSearch: problem " ++ problemNum args
  putStrLn $ "  input:     " ++ inputPath args
  putStrLn $ "  output:    " ++ outDir
  putStrLn $ "  obstacles: " ++ show (length obstacles)
  putStrLn $ "  attempts:  " ++ show (maxAttempts args)
  putStrLn $ "  nodes:     " ++ show (Map.size (Graphs.graphNodes tree))
  case result of
    Left err -> putStrLn $ "  result:    FAILED — " ++ err
    Right path -> putStrLn $ "  result:    OK — path length " ++ show (length path)

-------------------------------------------------------------------------------
-- CLI parsing
-------------------------------------------------------------------------------

-- Usage: ./rrtSearch [-a N] [INPUTDIR] [OUTPUTDIR] [PROBLEMNUM]
parse :: [String] -> IO Args
parse argv = case argv of
  ["-h"] -> usage >> exit >> return defaultArgs
  ["--help"] -> usage >> exit >> return defaultArgs
  ["-v"] -> version >> exit >> return defaultArgs
  ["--version"] -> version >> exit >> return defaultArgs
  _ -> parseFlags argv defaultArgs

-- | scan for -a/--attempts before consuming positional arguments
--   this allows the flag to appear anywhere in the argument list
parseFlags :: [String] -> Args -> IO Args
parseFlags [] args = return args
parseFlags ("-a" : n : rest) args = case reads n of
  [(i, "")] | i > 0 -> parseFlags rest args {maxAttempts = i}
  _ -> putStrLn ("Error: -a/--attempts requires a positive integer. Got: " ++ n) >> die >> return defaultArgs
parseFlags ("--attempts" : n : rest) args = case reads n of
  [(i, "")] | i > 0 -> parseFlags rest args {maxAttempts = i}
  _ -> putStrLn ("Error: -a/--attempts requires a positive integer. Got: " ++ n) >> die >> return defaultArgs
parseFlags ("-a" : []) _ =
  putStrLn "Error: -a/--attempts requires an argument" >> die >> return defaultArgs
parseFlags ("--attempts" : []) _ =
  putStrLn "Error: -a/--attempts requires an argument" >> die >> return defaultArgs
parseFlags positionals args = parsePositional positionals args

-- | parse up to three positional arguments,
--   fills in defaults for any omitted
parsePositional :: [String] -> Args -> IO Args
parsePositional [] args = return args
parsePositional [i] args = return args {inputPath = i}
parsePositional [i, o] args = return args {inputPath = i, outputPath = o}
parsePositional (i : o : p : _) args =
  let args' = args {inputPath = i, outputPath = o, problemNum = p}
   in validate args'
parsePositional _ args = return args

-- | validate that problem number "looks like" a two-digit numeric string
validate :: Args -> IO Args
validate args =
  if all (`elem` ("0123456789" :: String)) (problemNum args)
    && length (problemNum args) == 2
    then return args
    else do
      putStrLn $ "Error: PROBLEMNUM must be a 2-digit number (e.g. 01, 05). Got: " ++ problemNum args
      die
      return defaultArgs

-------------------------------------------------------------------------------
-- Info flags
-------------------------------------------------------------------------------

usage :: IO ()
usage = do
  putStrLn "Usage: rrtSearch [-a N] [INPUTDIR] [OUTPUTDIR] [PROBLEMNUM]"
  putStrLn ""
  putStrLn "Arguments:"
  putStrLn "  INPUTDIR"
  putStrLn "          Directory with obstacles file (`obstacles.txt`) and problem file (`p<##>.json`)"
  putStrLn "          Default: `./data/`"
  putStrLn ""
  putStrLn "  OUTDIR"
  putStrLn "          Directory to save solution files (`<##>/path.csv`, `<##>/search_tree.csv`)"
  putStrLn "          Solution files will be placed in a subdirectory corresponding to the problem number"
  putStrLn "          Default: `./outputs/`"
  putStrLn ""
  putStrLn "  PROBLEMNUM"
  putStrLn "          Number of problem for which to calculate RRT path"
  putStrLn "          This should correspond to a valid problem file (`p<##>.json`) in the provided [INPUTDIR]"
  putStrLn "          Problem numbers and associated files should be numerically padded to 2 digits"
  putStrLn "          Default: `01`"
  putStrLn ""
  putStrLn "Options:"
  putStrLn "  -a N, --attempts N"
  putStrLn "          Maximum number of RRT iterations before giving up"
  putStrLn "          Default: 100000"
  putStrLn ""
  putStrLn "  -h, --help"
  putStrLn "          Print this help menu"
  putStrLn ""
  putStrLn "  -v, --version"
  putStrLn "          Print version"

version :: IO ()
version = putStrLn "rrtSearch 0.1 (GHC 9.10)"

-------------------------------------------------------------------------------
-- Helpers
-------------------------------------------------------------------------------

exit :: IO ()
exit = System.Exit.exitWith System.Exit.ExitSuccess

die :: IO ()
die = System.Exit.exitWith (System.Exit.ExitFailure 1)
