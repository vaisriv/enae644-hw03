= problem statement

- the RRT code from the previous assignment, add functionality for "1/2 car"-like dynamics and a finite volume robot

== given

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

== constraints

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

== expected outputs
