-- Cartographer 2D mapping for the ARCS F1Tenth car.
-- Based on cartographer_ros' revo_lds.lua (small 2D lidar, no IMU in Cartographer).
--
-- "arcsf1tenth" in the frame names is a placeholder: the launch files replace it
-- with the namespace launch argument (Lua cannot read launch arguments).
--
--   lidar scan ────────────────┐
--   odometry/estimated (EKF) ──┴──► Cartographer ──► TF map → odom, map (occupancy grid)
--
-- The IMU is NOT used here: it is already fused in the EKF (odometry/estimated),
-- using it twice would count it twice.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "arcsf1tenth/map",
  tracking_frame = "arcsf1tenth/base_footprint",
  published_frame = "arcsf1tenth/odom",     -- Cartographer publishes map -> odom
  odom_frame = "arcsf1tenth/odom",
  provide_odom_frame = false,               -- the EKF already provides odom -> base_footprint
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,
  use_odometry = true,                      -- odom topic, remapped to odometry/estimated
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- RPLidar C1 (arcsf1tenth_description/config/sensors.yaml)
-- min_range: ignore points closer than the car body. The lidar sits 0.11 m ahead of the
-- rear axle; the car outline reaches ~0.23 m (rear corners) to ~0.31 m (front corners)
-- from it, so a real lidar can hit the car itself (the simulated one does not). A wall
-- closer than 0.3 m to the lidar is a crash anyway.
TRAJECTORY_BUILDER_2D.min_range = 0.3
-- "No return" beams: scan_for_slam.py (scan -> scan/slam) turns inf into the lidar
-- range_max (12.0). max_range just below it makes Cartographer treat those beams as
-- "no wall" and mark missing_data_ray_length of free space along them. Without this,
-- inf beams were dropped: on straights longer than 12 m nothing marked the cells along
-- the driven path as free, and they stayed light grey.
-- 5 m, not 11.9: the RPLidar C1 sees dark (10 % reflectivity) surfaces only up to ~6 m,
-- a longer free ray could erase such a wall on the real car.
TRAJECTORY_BUILDER_2D.max_range = 11.9
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1   -- one message = one full 360 deg scan
TRAJECTORY_BUILDER_2D.use_imu_data = false

-- Local scan matching: the correlative matcher is more robust with a slow (10 Hz) lidar
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35

-- Global optimization / loop closure
POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimize_every_n_nodes = 35
POSE_GRAPH.constraint_builder.min_score = 0.6
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.8

return options
