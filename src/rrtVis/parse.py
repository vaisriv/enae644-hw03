"""rrtVis.parse — load input data and RRT output CSVs"""

import csv
import json
import os
from dataclasses import dataclass

###############################################################################
# Data classes (mirror for Haskell types)
###############################################################################


@dataclass
class Obstacle:
    x: float
    y: float
    radius: float


@dataclass
class Workspace:
    x_min: float
    x_max: float
    y_min: float
    y_max: float


@dataclass
class Goal:
    x: float
    y: float
    radius: float


@dataclass
class Problem:
    workspace: Workspace
    start: tuple[float, float]
    goal: Goal
    epsilon: float


@dataclass
class TreeNode:
    node_id: int
    x: float
    y: float
    parent_id: int  # -1 for root


###############################################################################
# Loaders
###############################################################################


def load_obstacles(data_dir: str) -> list[Obstacle]:
    """parse `obstacles.txt` and return a list of Obstacle instances

    format:
        num_obstacles
        x_1, y_1, radius_1
        ...
    """
    path = os.path.join(data_dir, "obstacles.txt")
    obstacles = []
    with open(path) as f:
        lines = [l.strip() for l in f.readlines()]
    # first line is count — skip it, parse the rest
    for line in lines[1:]:
        if not line:
            continue
        parts = [p.strip() for p in line.split(",")]
        if len(parts) != 3:
            raise ValueError(f"malformed obstacle line: {line!r}")
        x, y, r = float(parts[0]), float(parts[1]), float(parts[2])
        obstacles.append(Obstacle(x=x, y=y, radius=r))
    return obstacles


def load_problem(data_dir: str, problem_num: str) -> Problem:
    """parse `pXX.json` and return a Problem instance"""
    path = os.path.join(data_dir, f"p{problem_num}.json")
    with open(path) as f:
        d = json.load(f)
    workspace = Workspace(
        x_min=d["workspace"]["x"][0],
        x_max=d["workspace"]["x"][1],
        y_min=d["workspace"]["y"][0],
        y_max=d["workspace"]["y"][1],
    )
    start = (float(d["start"]["x"]), float(d["start"]["y"]))
    goal = Goal(
        x=float(d["goal"]["x"]),
        y=float(d["goal"]["y"]),
        radius=float(d["goal"]["radius"]),
    )
    epsilon = float(d["motion"]["epsilon"])
    return Problem(workspace=workspace, start=start, goal=goal, epsilon=epsilon)


def load_search_tree(output_dir: str) -> list[TreeNode]:
    """read `search_tree.csv` (node_id,x,y,parent_id) and return nodes in order"""
    path = os.path.join(output_dir, "search_tree.csv")
    nodes = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            nodes.append(
                TreeNode(
                    node_id=int(row["node_id"]),
                    x=float(row["x"]),
                    y=float(row["y"]),
                    parent_id=int(row["parent_id"]),
                )
            )
    return nodes


def load_path(output_dir: str) -> list[tuple[float, float]] | str:
    """read `path.csv` and return either a list of (x, y) waypoints,
    or an error string if the file contains an ERROR: message"""
    path = os.path.join(output_dir, "path.csv")
    with open(path, newline="") as f:
        first_line = f.readline().strip()
        # error case — return the message as a plain string
        if first_line.startswith("ERROR:"):
            return first_line
        # success case — re-read with csv.DictReader (first line was the header)
        f.seek(0)
        reader = csv.DictReader(f)
        return [(float(row["x"]), float(row["y"])) for row in reader]
