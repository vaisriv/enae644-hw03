module Main where

import GHC.Generics
import qualified Graphs
  ( Edge (..),
    Graph (..),
    Node (..),
    printGraph,
  )
import qualified RRT
  ( Workspace (..),
  -- Obstacle (..),
  -- Problem (..),
  -- rrt,
  )
import qualified System.Environment
import qualified System.Exit
import qualified System.FilePath
  ( FilePath (..),
    (</>),
  )

-- args
data Args = Args
  { inputPath :: System.FilePath.FilePath,
    outputPath :: System.FilePath.FilePath
  }

-- main
main :: IO ()
main = do
  args <- System.Environment.getArgs >>= parse
  putStrLn "foobar"

-- CLI parsing
-- ./enae644-hw02 [INPUTDIR] [OUTPUTDIR] [PROBLEMNUM]
parse :: [String] -> IO Args
parse argv = case argv of
  ["-h"] -> usage >> exit >> return dummy
  ["--help"] -> usage >> exit >> return dummy
  ["-v"] -> version >> exit >> return dummy
  ["--version"] -> version >> exit >> return dummy
  _ -> parseFlags argv (Args "" "")
  where
    dummy = Args "" ""

-- info flags
usage :: IO ()
usage = do
  putStrLn "Usage: enae644-hw02 [INPUTDIR] [OUTPUTDIR] [PROBLEMNUM]"
  putStrLn ""
  putStrLn "Arguments:"
  putStrLn "  INPUTDIR"
  putStrLn "          Directory with obstacles file (`obstacles.txt`) and problem file (`p<##>.toml`)"
  putStrLn "          Default: `./data/`"
  putStrLn ""
  putStrLn "  OUTDIR"
  putStrLn "          Directory to save solution files (`<##>/path.csv`, `<##>/search_tree.csv`)"
  putStrLn "          Solution files will be placed in a subdirectory corresponding to the problem number"
  putStrLn "          Default: `./outputs/`"
  putStrLn ""
  putStrLn "  PROBLEMNUM"
  putStrLn "          Number of problem for which to calculate RRT path"
  putStrLn "          This should correspond to a valid problem file (`p<##>.toml`) in the provided [INPUTDIR]"
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
version = putStrLn "enae644-hw02 0.1 (GHC 9.10)"

-- flag parsing
parseFlags :: [String] -> Args -> IO Args
parseFlags [] args = validate args
parseFlags (flag : _) _ = do
  putStrLn $ "Unknown flag: " ++ flag
  usage
  die
  return (Args "" "")

-- flag validation
validate :: Args -> IO Args
validate args
  | otherwise = return args

exit :: IO ()
exit = System.Exit.exitWith System.Exit.ExitSuccess

die :: IO ()
die = System.Exit.exitWith (System.Exit.ExitFailure 1)
