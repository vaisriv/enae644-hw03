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
    start_theta: float  # orientation at start
    goal: Goal
    epsilon: float


@dataclass
class TreeNode:
    node_id: int
    x: float
    y: float
    theta: float
    v: float
    w: float
    parent_id: int  # -1 for root


@dataclass
class PathNode:
    t: float  # time
    x: float
    y: float
    theta: float
    v: float
    w: float
    a: float  # acceleration control
    gamma: float  # steering acceleration control


@dataclass
class RobotShape:
    points: list[tuple[float, float]]  # all robot points
    hull: list[tuple[float, float]]  # convex hull points


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
        lines = [line.strip() for line in f.readlines()]
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
    import math
    start_theta = float(d["start"]["theta_pi_rad"]) * math.pi  # convert from pi-radians to radians
    goal = Goal(
        x=float(d["goal"]["x"]),
        y=float(d["goal"]["y"]),
        radius=float(d["goal"]["radius"]),
    )
    epsilon = float(d["motion"]["epsilon"])
    return Problem(workspace=workspace, start=start, start_theta=start_theta, goal=goal, epsilon=epsilon)


def load_robot(data_dir: str) -> RobotShape:
    """parse `robot.txt` and return robot shape with all points and convex hull

    format:
        x_1, y_1
        x_2, y_2
        ...

    Note: We compute the convex hull here using Graham scan to match Haskell implementation
    """
    path = os.path.join(data_dir, "robot.txt")
    points = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = [p.strip() for p in line.split(",")]
            if len(parts) != 2:
                raise ValueError(f"malformed robot line: {line!r}")
            x, y = float(parts[0]), float(parts[1])
            points.append((x, y))

    # Compute convex hull using Graham scan
    hull = _convex_hull(points)
    return RobotShape(points=points, hull=hull)


def _convex_hull(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Compute convex hull using Graham scan algorithm"""
    import math

    if len(points) < 3:
        return points

    # Find point with lowest y-coordinate (leftmost if tie)
    p0 = min(points)

    # Sort points by polar angle with respect to p0
    def angle_from(p):
        return math.atan2(p[1] - p0[1], p[0] - p0[0])

    sorted_points = sorted([p for p in points if p != p0], key=angle_from)

    # Graham scan
    def ccw(p1, p2, p3):
        """Counter-clockwise test"""
        return (p2[0] - p1[0]) * (p3[1] - p1[1]) - (p2[1] - p1[1]) * (p3[0] - p1[0]) > 0

    hull = [p0]
    for p in sorted_points:
        while len(hull) >= 2 and not ccw(hull[-2], hull[-1], p):
            hull.pop()
        hull.append(p)

    return hull


def load_search_tree(output_dir: str) -> list[TreeNode]:
    """read `search_tree.csv` (node_id,x,y,theta,v,w,parent_id) and return nodes in order"""
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
                    theta=float(row["theta"]),
                    v=float(row["v"]),
                    w=float(row["w"]),
                    parent_id=int(row["parent_id"]),
                )
            )
    return nodes


def load_path(output_dir: str) -> list[PathNode] | str:
    """read `path.csv` and return either a list of PathNode objects,
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
        return [
            PathNode(
                t=float(row["t"]),
                x=float(row["x"]),
                y=float(row["y"]),
                theta=float(row["theta"]),
                v=float(row["v"]),
                w=float(row["w"]),
                a=float(row["a"]),
                gamma=float(row["gamma"]),
            )
            for row in reader
        ]
