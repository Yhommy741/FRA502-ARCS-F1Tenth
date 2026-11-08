cmake_minimum_required(VERSION 3.8)
project(arcsf1tenth_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Interfaces only: every other package depends on this one
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FrenetState.msg"
  "msg/Waypoint.msg"
  "msg/Trajectory.msg"
  "msg/Obstacle.msg"
  "msg/ObstacleArray.msg"
  "srv/PlanAvoidance.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
