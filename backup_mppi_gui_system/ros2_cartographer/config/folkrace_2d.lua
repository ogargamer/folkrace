include "map_builder.lua"
include "trajectory_builder.lua"

-- =============================================================================
--  GOOGLE CARTOGRAPHER 2D SLAM TUNED FOR 6x6m HIGH-SPEED FOLKRACE CAR ON PI 5
--  Sensors: ms200k 360-LiDAR (450 pts) + BNO08X IMU (Gyro + Accel + Orientation)
-- =============================================================================

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",            -- Tracks IMU at Center of Gravity for high-freq rotation
  published_frame = "base_link",          -- Direct transform from map to base_link
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  publish_tracked_pose = true,            -- Publishes /tracked_pose for real-time MPPI vehicle localization
  use_odometry = false,                   -- Set true if high-precision wheel odometry is fed
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,         -- 200 Hz fast pose output for racing
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,                -- 100% IMU packets processed for slip/yaw detection
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4    -- Utilizes all 4 Cortex-A76 cores on Pi 5

-- -----------------------------------------------------------------------------
-- 1. LOCAL SLAM & SENSOR WEIGHT PREFERENCES (LiDAR vs IMU)
-- -----------------------------------------------------------------------------
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.min_range = 0.05
TRAJECTORY_BUILDER_2D.max_range = 4.50
TRAJECTORY_BUILDER_2D.min_z = -0.10
TRAJECTORY_BUILDER_2D.max_z = 0.50
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 0.0

-- Sensor weighting: Prioritize sharp LiDAR scan matching with strong IMU yaw stabilization
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 20.0   -- High LiDAR geometric trust
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.0      -- Translation trust
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.0         -- Heavy rotational penalty to prevent scan yaw jump
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 5.0

-- Real-time Correlative Scan Matcher (Ceres seed initializer for dynamic motion)
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.10   -- 10 cm search window in 6x6m
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(15.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- -----------------------------------------------------------------------------
-- 2. HIT / MISS PROBABILITIES & DYNAMIC OBSTACLE DELETION (Grid Model)
-- -----------------------------------------------------------------------------
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.grid_type = "PROBABILITY_GRID"
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.02          -- 20 mm high-precision resolution

-- Probability Updates:
-- hit_probability: How quickly a solid wall/pillar is confirmed (0.55 -> ~2-3 hits to mark solid)
-- miss_probability: How aggressively empty laser rays clear out transient/ghost hits (0.47 = fast clearance)
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.58
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.46
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.insert_free_space = true

-- Submap size: 60 range insertions per submap (~15 submaps for entire 6x6 circuit)
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 60

-- Motion filter: Inserts on motion (dx >= 4cm or dtheta >= 2 deg); does not spam submaps when stationary
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5.0
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.04
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(2.0)

-- -----------------------------------------------------------------------------
-- 3. GLOBAL POSE GRAPH & LOOP CLOSURE OPTIMIZATION (Loop Closing on 6x6 Circuit)
-- -----------------------------------------------------------------------------
POSE_GRAPH.optimize_every_n_nodes = 25                                   -- Run loop closure every 25 submap nodes
POSE_GRAPH.constraint_builder.min_score = 0.60                           -- Score threshold to accept loop closures
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.65
POSE_GRAPH.constraint_builder.max_constraint_distance = 6.0              -- 6m diagonal bounds for 6x6m arena
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 3.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(180.)

-- Pose Graph Optimization Weights
POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimization_problem.acceleration_weight = 1e3
POSE_GRAPH.optimization_problem.rotation_weight = 3e5
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5

return options
