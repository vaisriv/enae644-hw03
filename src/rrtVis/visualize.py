"""rrtVis.visualize — render RRT results using matplotlib"""

import math
import matplotlib.figure
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import matplotlib.collections as mcollections
import numpy as np

from rrtVis.parse import Goal, Obstacle, Problem, TreeNode, PathNode, RobotShape

###############################################################################
# Palette
###############################################################################

BG = "#ffffff"  # figure background (white)
AXES_BG = "#f7f7f7"  # axes area (off-white)
WORKSPACE = "#ffffff"  # workspace fill
SPINE = "#cccccc"  # axes spines and workspace border
TICK = "#666666"  # tick marks and labels
LEGEND_BG = "#ffffff"  # legend background
LEGEND_EDGE = "#cccccc"  # legend border
LEGEND_TEXT = "#333333"  # legend label text
OBSTACLE = "#d9534f"  # obstacle fill (red — danger)
OBSTACLE_ED = "#a02020"  # obstacle edge (darker red)
TREE_EDGE = "#a8c4e0"  # search tree edges (light steel blue)
START_COL = "#e67e00"  # start marker (burnt orange)
START_EDGE = "#7a3c00"  # start marker edge (dark brown)
GOAL_FILL = "#5cb85c"  # goal region fill (green — target)
GOAL_EDGE = "#2e7d32"  # goal region edge (dark green)
PATH_COL = "#1a1a2e"  # solution path (near-black navy)
PATH_HALO = "#1a1a2e"  # path glow halo (same hue, controlled by alpha)
ERROR_TEXT = "#a02020"  # error message text
ERROR_BG = "#fff0f0"  # error message box background
ERROR_EDGE = "#d9534f"  # error message box border
ROBOT_POINT = "#3b5998"  # robot point cloud (medium blue)
ROBOT_HULL = "#1a237e"  # robot hull boundary (dark blue)
ROBOT_HULL_FILL = "#3b5998"  # robot hull fill (medium blue, with alpha)

###############################################################################
# Rendering
###############################################################################


def render(
    obstacles: list[Obstacle],
    problem: Problem,
    tree: list[TreeNode],
    path: list[PathNode] | str,
    robot: RobotShape,
) -> matplotlib.figure.Figure:
    """render the full RRT result into a matplotlib Figure and return it

    if `path` is a string it is treated as an error message and displayed
    as an annotation on the plot instead of drawing a path
    """
    # Create figure with two subplots: main plot (left) and robot inset (right)
    fig = plt.figure(figsize=(10, 8), facecolor=BG)
    ax_main = plt.subplot2grid((1, 4), (0, 0), colspan=3, fig=fig)
    ax_robot = plt.subplot2grid((1, 4), (0, 3), fig=fig)

    ax_main.set_facecolor(AXES_BG)
    ax_robot.set_facecolor(AXES_BG)

    # draw layers bottom-to-top on main plot
    draw_workspace(ax_main, problem)
    draw_obstacles(ax_main, obstacles)
    draw_tree(ax_main, tree)
    draw_goal(ax_main, problem.goal)
    draw_start(ax_main, problem.start, problem.start_theta)

    if isinstance(path, str):
        draw_error(ax_main, path)
    else:
        draw_path(ax_main, path)
        # Draw robot at goal position with final orientation from path
        if path:
            final_pose = path[-1]
            draw_robot_at_pose(ax_main, robot, final_pose.x, final_pose.y, final_pose.theta, alpha=0.5)

    # main axes styling
    ws = problem.workspace
    margin = (ws.x_max - ws.x_min) * 0.02
    ax_main.set_xlim(ws.x_min - margin, ws.x_max + margin)
    ax_main.set_ylim(ws.y_min - margin, ws.y_max + margin)
    ax_main.set_aspect("equal")

    for spine in ax_main.spines.values():
        spine.set_edgecolor(SPINE)
    ax_main.tick_params(colors=TICK, labelsize=8)
    ax_main.xaxis.label.set_color(TICK)
    ax_main.yaxis.label.set_color(TICK)

    # draw robot shape in inset
    draw_robot_shape(ax_robot, robot)

    # legend on main plot
    legend_handles = [
        mpatches.Patch(facecolor=OBSTACLE, edgecolor=OBSTACLE_ED, label="Obstacle"),
        mpatches.Patch(facecolor=GOAL_FILL, edgecolor=GOAL_EDGE, label="Goal region"),
        plt.Line2D([0], [0], color=TREE_EDGE, linewidth=0.6, label="Search tree"),
        plt.Line2D(
            [0],
            [0],
            color=START_COL,
            marker="*",
            markersize=9,
            linewidth=0,
            label="Start",
        ),
    ]
    if isinstance(path, list):
        legend_handles.append(
            plt.Line2D([0], [0], color=PATH_COL, linewidth=2.0, label="Path")
        )
    ax_main.legend(
        handles=legend_handles,
        loc="upper right",
        fontsize=7,
        facecolor=LEGEND_BG,
        edgecolor=LEGEND_EDGE,
        labelcolor=LEGEND_TEXT,
        framealpha=0.9,
    )

    fig.tight_layout()
    return fig


###############################################################################
# Drawing helpers
###############################################################################


def draw_workspace(ax, problem: Problem) -> None:
    """draw workspace boundary as a filled rectangle"""
    ws = problem.workspace
    rect = mpatches.Rectangle(
        (ws.x_min, ws.y_min),
        ws.x_max - ws.x_min,
        ws.y_max - ws.y_min,
        linewidth=1.2,
        edgecolor=SPINE,
        facecolor=WORKSPACE,
        zorder=0,
    )
    ax.add_patch(rect)


def draw_obstacles(ax, obstacles: list[Obstacle]) -> None:
    """draw filled obstacle circles"""
    for obs in obstacles:
        circle = mpatches.Circle(
            (obs.x, obs.y),
            obs.radius,
            linewidth=0.8,
            edgecolor=OBSTACLE_ED,
            facecolor=OBSTACLE,
            alpha=0.85,
            zorder=2,
        )
        ax.add_patch(circle)


def draw_tree(ax, tree: list[TreeNode]) -> None:
    """draw all edges in the search tree as thin lines using a LineCollection
    for performance — avoids one matplotlib call per edge"""
    # build a node_id -> (x, y) lookup
    node_map: dict[int, tuple[float, float]] = {n.node_id: (n.x, n.y) for n in tree}
    segments = []
    for node in tree:
        if node.parent_id == -1:
            continue
        parent = node_map.get(node.parent_id)
        if parent is not None:
            segments.append([(parent[0], parent[1]), (node.x, node.y)])

    if segments:
        lc = mcollections.LineCollection(
            segments,
            colors=TREE_EDGE,
            linewidths=0.5,
            alpha=0.9,
            zorder=1,
        )
        ax.add_collection(lc)


def draw_goal(ax, goal: Goal) -> None:
    """draw the goal region as a semi-transparent filled circle"""
    circle = mpatches.Circle(
        (goal.x, goal.y),
        goal.radius,
        linewidth=1.2,
        edgecolor=GOAL_EDGE,
        facecolor=GOAL_FILL,
        alpha=0.30,
        zorder=3,
    )
    ax.add_patch(circle)
    # crisp edge ring on top
    ring = mpatches.Circle(
        (goal.x, goal.y),
        goal.radius,
        linewidth=1.2,
        edgecolor=GOAL_EDGE,
        facecolor="none",
        zorder=4,
    )
    ax.add_patch(ring)


def draw_start(ax, start: tuple[float, float], theta: float) -> None:
    """draw the start position as a star marker with orientation arrow"""
    # Star marker at start position
    ax.plot(
        start[0],
        start[1],
        marker="*",
        markersize=12,
        color=START_COL,
        markeredgecolor=START_EDGE,
        markeredgewidth=0.6,
        zorder=5,
    )
    # Orientation arrow
    arrow_length = 3.0
    dx = arrow_length * math.cos(theta)
    dy = arrow_length * math.sin(theta)
    ax.arrow(
        start[0],
        start[1],
        dx,
        dy,
        head_width=1.5,
        head_length=1.0,
        fc=START_COL,
        ec=START_EDGE,
        linewidth=1.5,
        alpha=0.8,
        zorder=5,
    )


def draw_path(ax, path: list[PathNode]) -> None:
    """draw the solution path with curved trajectories following vehicle dynamics"""
    if not path:
        return

    # Import dynamics module for curved trajectory computation
    from rrtVis.dynamics import compute_curved_path

    # Compute curved trajectory points using RK4 integration
    curved_points = compute_curved_path(path, dt=0.01)

    if not curved_points:
        return

    # Extract x, y coordinates for curved trajectory
    xs_curve = [p[0] for p in curved_points]
    ys_curve = [p[1] for p in curved_points]

    # Draw curved trajectory with halo and main line
    # halo layer — wide, low-alpha pass for a subtle shadow effect on light bg
    ax.plot(xs_curve, ys_curve, color=PATH_HALO, linewidth=5.0, alpha=0.08, zorder=6)
    # main path line
    ax.plot(xs_curve, ys_curve, color=PATH_COL, linewidth=1.8, alpha=0.95, zorder=7)

    # Draw waypoint markers at original PathNode positions
    xs_waypoints = [p.x for p in path]
    ys_waypoints = [p.y for p in path]
    ax.scatter(xs_waypoints, ys_waypoints, color=PATH_COL, s=8, zorder=8)

    # Draw orientation arrows at path waypoints (every 3rd point to avoid clutter)
    arrow_length = 2.0
    for i, p in enumerate(path):
        if i % 3 == 0:  # subsample to avoid too many arrows
            dx = arrow_length * math.cos(p.theta)
            dy = arrow_length * math.sin(p.theta)
            ax.arrow(
                p.x,
                p.y,
                dx,
                dy,
                head_width=0.8,
                head_length=0.6,
                fc=PATH_COL,
                ec=PATH_COL,
                linewidth=1.0,
                alpha=0.5,
                zorder=7,
            )


def draw_error(ax, message: str) -> None:
    """display an error message centered on the plot"""
    ax.text(
        0.5,
        0.5,
        message,
        transform=ax.transAxes,
        ha="center",
        va="center",
        fontsize=10,
        color=ERROR_TEXT,
        fontfamily="monospace",
        bbox=dict(
            boxstyle="round,pad=0.6",
            facecolor=ERROR_BG,
            edgecolor=ERROR_EDGE,
            alpha=0.95,
        ),
        zorder=10,
        wrap=True,
    )


def draw_robot_at_pose(ax, robot: RobotShape, x: float, y: float, theta: float, alpha: float = 0.3) -> None:
    """draw robot shape at a given pose (x, y, theta) on the workspace"""
    import math

    # Transform all hull points to world frame
    transformed_hull = []
    for px, py in robot.hull:
        wx = x + px * math.cos(theta) - py * math.sin(theta)
        wy = y + px * math.sin(theta) + py * math.cos(theta)
        transformed_hull.append((wx, wy))

    # Draw filled polygon for convex hull
    if len(transformed_hull) >= 3:
        polygon = mpatches.Polygon(
            transformed_hull,
            closed=True,
            facecolor=ROBOT_HULL_FILL,
            edgecolor=ROBOT_HULL,
            linewidth=1.0,
            alpha=alpha,
            zorder=4,
        )
        ax.add_patch(polygon)


def draw_robot_shape(ax, robot: RobotShape) -> None:
    """draw robot shape inset showing point cloud and convex hull boundary"""
    # Draw all points as scatter
    xs = [p[0] for p in robot.points]
    ys = [p[1] for p in robot.points]
    ax.scatter(xs, ys, color=ROBOT_POINT, s=12, alpha=0.6, zorder=2, label="Robot points")

    # Draw convex hull as polygon
    if len(robot.hull) >= 3:
        hull_xs = [p[0] for p in robot.hull] + [robot.hull[0][0]]  # close the polygon
        hull_ys = [p[1] for p in robot.hull] + [robot.hull[0][1]]
        ax.plot(hull_xs, hull_ys, color=ROBOT_HULL, linewidth=2.0, zorder=3, label="Convex hull")
        ax.fill(hull_xs, hull_ys, color=ROBOT_HULL_FILL, alpha=0.15, zorder=1)

    # Styling
    ax.set_aspect("equal")
    ax.set_title("Robot Shape", fontsize=9, color=TICK, pad=8)
    ax.tick_params(colors=TICK, labelsize=7)
    for spine in ax.spines.values():
        spine.set_edgecolor(SPINE)
    ax.grid(True, alpha=0.2, color=SPINE, linewidth=0.5)
    ax.legend(fontsize=6, loc="upper right", framealpha=0.9)
