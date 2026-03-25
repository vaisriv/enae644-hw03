= problem statement

- using the RRT code from the previous assignment as a base, add functionality for "1/2 car"-like dynamics and a finite volume robot

== system definition

=== "1/2 car"-like model

- configuration space: $C = R^2 times S times R^2$
    - location: $(x, y) in R^2, R = 50$
    - orientation: $theta in S, S = [0, 2 pi]$
    - translation velocity: $|v| lt.eq v_max = 5, v in R$
    - steering velocity: $|w| lt.eq w_max = pi/2, w in R$
- control inputs: $U = (a, gamma)$
    - forward acceleration: $|a| lt.eq a_max = 2$
    - steering acceleration: $|gamma| lt.eq gamma_max = pi/2$
    - assume constant $a, gamma$ along each local trajectory (i.e. each edge)
- workspace: $W = R^2$
- dynamics: $dif/(dif t) vec(delim: "[", x, y, theta, v, w) = vec(delim: "[", v cos theta, v sin theta, w, a, gamma)$

=== finite volume robot

- robot is defined by a set of points
- each point needs to be checked vs. obstacles
- robot position:
    - $mat(
        delim: "[",
        x_1, y_1;
        x_2, y_2;
        dots.v, dots.v;
        x_m, y_m;
    )$
    - assuming center of robot is at $(x, y, theta) = (0, 0, 0)$
- subsample along the trajectories you generate at a higher resolution:
    - for each edge, remember the trajectory associated with it at a resolution s.t. consecutive points are no further apart than the chosen $delta = 0.5$

== constraint summary

- points in the configuration space have the form $(x, y, theta, v, w)$
- position value $x$ is valid on the range $[-50, 50]$
- position value $y$ is valid on the range $[-50, 50]$
- rotation $theta$ goes from $[0, 2 pi]$
- forward velocity $v$ goes from $[-5, 5]$
- steering velocity $w$ goes from $[-pi/2, pi/2]$
- $C$-space Values outside this range are invalid
- forward acceleration $a = (dif v)/(dif t) in [-2, 2]$
- steering acceleration $gamma = (dif theta)/(dif t) in [-pi/2, pi/2]$
- trajectories need to be collision checked at least every delta distance with respect to obstacles
- problems start with the robot stopped (i.e. $v = 0, w = 0$)
- the problem may end with the robot still in motion (i.e. non-zero $(v, w, theta)$)
- assume $delta lt.eq 0.5$
- $"start" = (x_0, y_0, theta_0, v_0, w_0)$, and assume:
    - $x_0, y_0$ are given
    - $"start"$ happens at $t_0$
    - $theta_0 = 0$
    - $v_0 = 0$
    - $w_0 = 0$
- similarly, $"goal" = (x_n, y_n, theta_n, v_n, w_n)$ s.t. $x_n, y_n$ are in the goal region (goal region has the form $x, y, "radius"$)

== expected outputs

- for each problem:
    - graphical representation
        - workspace with obstacles drawn
        - paths calculated
        - robot bounding box at starting location
    - csv file
        - each line is $t_i, x_i, y_i, theta_i, v_i, w_i, a_i, gamma_i$, where $t_i, x_i, y_i, theta_i$ are values at trajectory $i$ start, and $v_i, u_i$ are the controls applied from time $t_i -> t_(i+1)$ to make the robot follow trajectory $i$
        - note that the path consists of the sequence of trajectories $i = [0, 1, ..., n]$
- a short write-up (about a paragraph) describing the solution to the dynamic constraints (e.g. detail the implementation/process of solving the 2P-BVP)
