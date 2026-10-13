cmake_minimum_required(VERSION 3.5)
project(arcsf1tenth_tracks)

find_package(ament_cmake REQUIRED)

# Data-only package: track worlds, maps and racelines
install(DIRECTORY
  f1tenth_tracks
  lab_tracks
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
