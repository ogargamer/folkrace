include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = true,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.5,
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
MAP_BUILDER.num_background_threads = 4

-- Trajectory Builder 2D (15mm Grid)
TRAJECTORY_BUILDER_2D.min_range = 0.12
TRAJECTORY_BUILDER_2D.max_range = 3.9
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 0.0
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10.0
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.01

-- Ceres Scan Matcher (Local SLAM)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 52.5
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 30.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 30

-- Real-Time Correlative Scan Matcher
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(12.0)

-- Motion Filter (Spatial Gating)
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.02
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.5)
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 1.0

-- Submaps (Locked Sub36 + Midpoint Active Carve)
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 36
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.grid_type = "PROBABILITY_GRID"
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.02
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.78
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.38

-- Pose Graph Global SLAM
POSE_GRAPH.optimize_every_n_nodes = 2
POSE_GRAPH.global_constraint_search_after_n_seconds = 0.0
POSE_GRAPH.constraint_builder.sampling_ratio = 1.0
POSE_GRAPH.global_sampling_ratio = 1.0
POSE_GRAPH.constraint_builder.max_constraint_distance = 25.0
POSE_GRAPH.constraint_builder.min_score = 0.36
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.36

-- Fast Correlative Scan Matcher
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 3.5
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(25.0)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 5

-- Optimization Problem Weights
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 500000.0
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 5000000.0
POSE_GRAPH.optimization_problem.huber_scale = 10.0
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 60

return options
