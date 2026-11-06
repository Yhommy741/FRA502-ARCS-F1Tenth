# A path with speeds for the controller (pure_pursuit trajectory_topic) or the planners.
std_msgs/Header header      # frame_id = map frame
bool closed                 # true = closed loop (the full global path), false = open segment
Waypoint[] points           # empty = no valid trajectory (planner found none)
