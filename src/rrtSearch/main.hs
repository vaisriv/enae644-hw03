{-# LANGUAGE OverloadedStrings #-}

module Main where

import Data.Aeson
  ( FromJSON (..),
    eitherDecodeFileStrict,
    withObject,
    (.:),
  )
import qualified Graphs
  ( Graph (..),
    Node (..),
    emptyGraph,
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

-------------------------------------------------------------------------------
-- Args
-------------------------------------------------------------------------------

data Args = Args
  { inputPath :: System.FilePath.FilePath,
    outputPath :: System.FilePath.FilePath,
    problemNum :: String
  }

defaultArgs :: Args
defaultArgs =
  Args
    { inputPath = "./data",
      outputPath = "./outputs",
      problemNum = "01"
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
--     { "x": ..., "y": ... }
data JsonPoint = JsonPoint
  { jpX :: Double,
    jpY :: Double
  }

instance FromJSON JsonPoint where
  parseJSON = withObject "Point" $ \o ->
    JsonPoint <$> o .: "x" <*> o .: "y"

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

-- | parse `pXX.json` into a Problem
--   obstacles are passed in separately (parsed from `obstacles.txt`)
parseProblem :: System.FilePath.FilePath -> [RRT.Obstacle] -> IO RRT.Problem
parseProblem path obs = do
  result <- eitherDecodeFileStrict path
  case result of
    Left err -> fail $ "failed to parse problem file: " ++ err
    Right jp -> return $ toProblem jp
  where
    toProblem jp =
      RRT.Problem
        { RRT.problemWorkspace = toWorkspace (jpWorkspace jp),
          RRT.problemStart = (jpX (jpStart jp), jpY (jpStart jp)),
          RRT.problemGoal = toGoal (jpGoal jp),
          RRT.problemMotion = toMotion (jpMotion jp),
          RRT.problemObstacles = obs
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
-- Main
-------------------------------------------------------------------------------

main :: IO ()
main = do
  args <- System.Environment.getArgs >>= parse
  let probFile = inputPath args System.FilePath.</> ("p" ++ problemNum args ++ ".json")
      obsFile = inputPath args System.FilePath.</> "obstacles.txt"
      outDir = outputPath args System.FilePath.</> problemNum args
  -- verify inputs exist
  probExists <- System.Directory.doesFileExist probFile
  obsExists <- System.Directory.doesFileExist obsFile
  if not probExists
    then putStrLn ("Error: problem file not found: " ++ probFile) >> die
    else return ()
  if not obsExists
    then putStrLn ("Error: obstacles file not found: " ++ obsFile) >> die
    else return ()
  -- ensure output directory exists
  System.Directory.createDirectoryIfMissing True outDir
  -- parse inputs
  obstacles <- parseObstacles obsFile
  problem <- parseProblem probFile obstacles
  -- TODO:
  -- (step 5): run RRT
  -- (step 6): write outputs
  putStrLn $ "rrtSearch: problem " ++ problemNum args
  putStrLn $ "  input:     " ++ inputPath args
  putStrLn $ "  output:    " ++ outDir
  putStrLn $ "  obstacles: " ++ show (length obstacles)
  putStrLn $ "  problem:   " ++ show problem

-------------------------------------------------------------------------------
-- CLI parsing
-------------------------------------------------------------------------------

-- Usage: ./rrtSearch [INPUTDIR] [OUTPUTDIR] [PROBLEMNUM]
parse :: [String] -> IO Args
parse argv = case argv of
  ["-h"] -> usage >> exit >> return defaultArgs
  ["--help"] -> usage >> exit >> return defaultArgs
  ["-v"] -> version >> exit >> return defaultArgs
  ["--version"] -> version >> exit >> return defaultArgs
  _ -> parsePositional argv defaultArgs

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
  putStrLn "Usage: rrtSearch [INPUTDIR] [OUTPUTDIR] [PROBLEMNUM]"
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
