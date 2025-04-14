include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0, -- LiDAR 사용 안 함
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1, -- ZED XM의 PointCloud2 사용
  publish_to_tf = true,
  lookup_transform_timeout_sec = 0.1,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 100e-3,
  trajectory_publish_period_sec = 100e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio= 0.8, --0.2에서 0.8로 수정함.
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 0.5,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.min_range = 0.4
TRAJECTORY_BUILDER_2D.max_range = 15.5
TRAJECTORY_BUILDER_2D.min_z = 0.0
TRAJECTORY_BUILDER_2D.max_z = 0.5
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 15.
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true 
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 20
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40

TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 9.81
POSE_GRAPH.optimization_problem.log_solver_summary = true


-- ZED XM의 PointCloud2를 직접 사용
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 15.
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.1  -- PointCloud의 해상도 조정
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.2


POSE_GRAPH.constraint_builder.min_score = 0.7
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.75

-- 루프 클로저 
POSE_GRAPH.optimize_every_n_nodes = 5 --기존에는 10 
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7


return options