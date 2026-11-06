# Car state in the Frenet frame of the global path (frenet_node).
#   s  = distance along the path, d = sideways offset from it (left of the path = positive)
std_msgs/Header header      # frame_id = frame of the path (map)
float64 s                   # [m] arc length along the path, 0 .. path_length
float64 d                   # [m] lateral offset, + = left
float64 s_dot               # [m/s] speed along the path
float64 d_dot               # [m/s] sideways speed
float64 yaw_error           # [rad] car heading - path heading
float64 speed               # [m/s] car speed (odometry)
float64 path_length         # [m] length of the closed path
uint32 index                # nearest point of the global trajectory
