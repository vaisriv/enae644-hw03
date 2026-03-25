# ENAE644 HW03

## rrtSearch & rrtVis

- [Problem Statement](https://github.com/vaisriv/enae644-hw03/blob/main/references/problems.pdf)
- [Submission Tarball](https://github.com/vaisriv/enae644-hw03/blob/main/submission.tar.gz)

### Implementation Decisions

The RRT algorithm was extended to handle 1/2-car-like dynamics using a random shooting approach to solve the Two-Point Boundary Value Problem (2P-BVP). For each steering query between two 5D states (x, y, θ, v, w), the algorithm generates 20 random control trajectories by sampling acceleration a ∈ [-2, 2] (as given), steering acceleration γ ∈ [-π/2, π/2] (also as given), and duration T ∈ [0.1, 5.0] seconds. Each trajectory is forward-integrated using RK4 numerical integration (dt=0.01) according to the 1/2-car dynamics: ẋ = v·cos(θ), ẏ = v·sin(θ), θ̇ = w, v̇ = a, ẇ = γ. The trajectory is accepted if it reaches within epsilon distance of the target state and is collision-free. Collision checking uses the convex hull of the robot shape during search for efficiency, then validates the final path with the full robot point cloud for safety. The RRT employs 10% goal biasing to improve convergence, sampling states from the goal region 10% of the time and random 5D states otherwise. Distance in configuration space uses weighted Euclidean metric with weights (1.0, 0.3, 0.2, 0.1) for position, orientation, linear velocity, and angular velocity respectively, prioritizing spatial proximity while accounting for kinematic differences.

## Contributors

- [Vai Srivastava](https://github.com/vaisriv)

## Acknowledgements

- RRT Algorithm implemented as per [ENAE644 Course Notes](https://uni.vaisriv.com/26s/ENAE644) and Lavalle's [Planning Algorithms (2006)](https://lavalle.pl/planning).
- Workspace Display is based off of Dr. Michael Otte's A\* graph search code.
