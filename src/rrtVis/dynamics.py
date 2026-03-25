"""rrtVis.dynamics — trajectory integration using RK4

This module implements RK4 (4th-order Runge-Kutta) numerical integration
for computing curved trajectories from vehicle dynamics, matching the Haskell
implementation in rrtSearch/Dynamics.hs.
"""

import math
from typing import NamedTuple

from rrtVis.parse import PathNode


class State5D(NamedTuple):
    """5D state for vehicle dynamics: position, orientation, velocities"""

    x: float
    y: float
    theta: float
    v: float  # linear velocity
    w: float  # angular velocity


def car_dynamics(
    x: float, y: float, theta: float, v: float, w: float, a: float, gamma: float
) -> tuple[float, float, float, float, float]:
    """Compute state derivatives from vehicle dynamics

    Args:
        x, y: Position
        theta: Orientation (radians)
        v: Linear velocity
        w: Angular velocity
        a: Linear acceleration (control input)
        gamma: Angular acceleration (control input)

    Returns:
        Tuple of (dx/dt, dy/dt, dtheta/dt, dv/dt, dw/dt)
    """
    return (
        v * math.cos(theta),  # dx/dt
        v * math.sin(theta),  # dy/dt
        w,  # dtheta/dt
        a,  # dv/dt
        gamma,  # dw/dt
    )


def integrate_rk4(state: State5D, control: tuple[float, float], dt: float) -> State5D:
    """Single RK4 integration step

    Advances the state by time dt using constant control inputs.
    Implements standard 4th-order Runge-Kutta integration matching
    the Haskell implementation in Dynamics.hs lines 95-142.

    Args:
        state: Current 5D state
        control: Tuple of (a, gamma) - acceleration control inputs
        dt: Time step

    Returns:
        New state after dt
    """
    a, gamma = control

    # k1 = f(state, control)
    dx1, dy1, dtheta1, dv1, dw1 = car_dynamics(
        state.x, state.y, state.theta, state.v, state.w, a, gamma
    )

    # k2 = f(state + dt/2 * k1, control)
    state2 = State5D(
        state.x + dt / 2 * dx1,
        state.y + dt / 2 * dy1,
        state.theta + dt / 2 * dtheta1,
        state.v + dt / 2 * dv1,
        state.w + dt / 2 * dw1,
    )
    dx2, dy2, dtheta2, dv2, dw2 = car_dynamics(
        state2.x, state2.y, state2.theta, state2.v, state2.w, a, gamma
    )

    # k3 = f(state + dt/2 * k2, control)
    state3 = State5D(
        state.x + dt / 2 * dx2,
        state.y + dt / 2 * dy2,
        state.theta + dt / 2 * dtheta2,
        state.v + dt / 2 * dv2,
        state.w + dt / 2 * dw2,
    )
    dx3, dy3, dtheta3, dv3, dw3 = car_dynamics(
        state3.x, state3.y, state3.theta, state3.v, state3.w, a, gamma
    )

    # k4 = f(state + dt * k3, control)
    state4 = State5D(
        state.x + dt * dx3,
        state.y + dt * dy3,
        state.theta + dt * dtheta3,
        state.v + dt * dv3,
        state.w + dt * dw3,
    )
    dx4, dy4, dtheta4, dv4, dw4 = car_dynamics(
        state4.x, state4.y, state4.theta, state4.v, state4.w, a, gamma
    )

    # next_state = state + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
    return State5D(
        state.x + dt / 6 * (dx1 + 2 * dx2 + 2 * dx3 + dx4),
        state.y + dt / 6 * (dy1 + 2 * dy2 + 2 * dy3 + dy4),
        state.theta + dt / 6 * (dtheta1 + 2 * dtheta2 + 2 * dtheta3 + dtheta4),
        state.v + dt / 6 * (dv1 + 2 * dv2 + 2 * dv3 + dv4),
        state.w + dt / 6 * (dw1 + 2 * dw2 + 2 * dw3 + dw4),
    )


def integrate_segment(
    start_node: PathNode, end_node: PathNode, dt: float = 0.01
) -> list[tuple[float, float]]:
    """Integrate trajectory between consecutive waypoints

    Each PathNode represents the START of a trajectory segment.
    The control inputs (a, gamma) from start_node are held constant
    until we reach end_node.

    Args:
        start_node: Starting waypoint with control inputs
        end_node: Ending waypoint (used only for duration via t field)
        dt: Integration time step (default 0.01 to match Haskell)

    Returns:
        List of (x, y) coordinate pairs along the curved trajectory
    """
    duration = end_node.t - start_node.t

    # Edge case: zero or negative duration
    if duration <= 0:
        return [(start_node.x, start_node.y), (end_node.x, end_node.y)]

    # Edge case: very small duration (less than one time step)
    if duration < dt:
        return [(start_node.x, start_node.y), (end_node.x, end_node.y)]

    # Initialize state from start_node
    state = State5D(
        x=start_node.x,
        y=start_node.y,
        theta=start_node.theta,
        v=start_node.v,
        w=start_node.w,
    )

    # Control inputs are constant for this segment
    control = (start_node.a, start_node.gamma)

    # Compute number of integration steps
    num_steps = math.ceil(duration / dt)

    # Integrate trajectory
    points = [(state.x, state.y)]
    for _ in range(num_steps):
        state = integrate_rk4(state, control, dt)
        points.append((state.x, state.y))

    return points


def compute_curved_path(
    path: list[PathNode], dt: float = 0.01
) -> list[tuple[float, float]]:
    """Generate curved trajectory from path waypoints

    Takes a list of PathNode waypoints and computes the full curved
    trajectory by integrating the vehicle dynamics between consecutive
    waypoints.

    Args:
        path: List of PathNodes from path.csv
        dt: Integration time step (default 0.01)

    Returns:
        List of (x, y) coordinate pairs along the entire curved path
    """
    # Edge case: empty path
    if not path:
        return []

    # Edge case: single waypoint
    if len(path) == 1:
        return [(path[0].x, path[0].y)]

    # Integrate each segment and collect points
    all_points = []

    for i in range(len(path) - 1):
        segment_points = integrate_segment(path[i], path[i + 1], dt)

        # Add segment points, avoiding duplicate at junction
        if i == 0:
            # First segment: include all points
            all_points.extend(segment_points)
        else:
            # Subsequent segments: skip first point (duplicate of previous segment's last point)
            all_points.extend(segment_points[1:])

    return all_points
