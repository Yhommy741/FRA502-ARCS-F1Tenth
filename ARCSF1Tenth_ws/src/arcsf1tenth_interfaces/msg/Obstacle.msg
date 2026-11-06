# Obstacle seen by the lidar that is not in the saved map (obstacle_detector).
uint32 id
float64 x                   # [m] center, map frame
float64 y
float64 radius              # [m] radius of a circle around all its points
float64 s_center            # [m] Frenet coordinates (global path)
float64 s_start             # [m] extent along the path (may wrap past path_length)
float64 s_end
float64 d_center            # [m] lateral, + = left
float64 d_min               # [m] lateral extent: right edge
float64 d_max               # [m] left edge
