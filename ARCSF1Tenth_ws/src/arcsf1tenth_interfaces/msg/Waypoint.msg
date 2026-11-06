# One point of a trajectory, in Cartesian (map frame) and Frenet (global path) coordinates.
float64 s                   # [m] arc length along the GLOBAL path
float64 d                   # [m] lateral offset from the global path, + = left
float64 x                   # [m]
float64 y                   # [m]
float64 psi                 # [rad] heading, 0 = +x, CCW positive
float64 kappa               # [1/m] curvature, + = turning left
float64 v                   # [m/s] target speed (0 = stop here)
float64 d_left              # [m] free space from the global path to the left wall (global path only)
float64 d_right             # [m] free space from the global path to the right wall (positive number)
