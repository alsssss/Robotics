include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = false,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  publish_tracked_pose = true,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.02,
  submap_publish_period_sec = 2.0,
  pose_publish_period_sec = 50e-3,
  trajectory_publish_period_sec = 50e-3,
  rangefinder_sampling_ratio = 0.5,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 0.5,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 16


TRAJECTORY_BUILDER_3D.submaps.num_range_data = 1e3
TRAJECTORY_BUILDER_3D.min_range = 1.0
TRAJECTORY_BUILDER_3D.max_range = 50.0

 

TRAJECTORY_BUILDER_3D.imu_gravity_time_constant = 0.5  




POSE_GRAPH.optimize_every_n_nodes = 1e4
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.constraint_builder.max_constraint_distance = 40.
POSE_GRAPH.constraint_builder.min_score = 0.9
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.9

POSE_GRAPH.optimization_problem.huber_scale = 1e3 






return options
