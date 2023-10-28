include 'map_builder.lua'
include 'trajectory_builder.lua'

options = {
    map_builder = MAP_BUILDER,
    trajectory_builder = TRAJECTORY_BUILDER,
    map_frame = 'map',
    tracking_frame = 'base_link',
    published_frame = 'base_link',
    odom_frame = 'odom',
    provide_odom_frame = false,
    use_odometry = false,
    use_nav_sat = false,
    use_landmarks = false,
    publish_frame_projected_to_2d = false,
    num_laser_scans = 1,
    num_multi_echo_laser_scans = 0,
    num_subdivisions_per_laser_scan = 1,
    num_point_clouds = 1,
    rangefinder_sampling_ratio = 1.0,
    rangefinder_sampling_ratio = 0.1,
    fixed_frame_pose_sampling_ratio = 1.0,
    imu_sampling_ratio = 1.0,
    landmarks_sampling_ratio = 1.0,
    lookup_transform_timeout_sec = 0.5,
    submap_publish_period_sec = 0.5,
    pose_publish_period_sec = 0.1,
    trajectory_publish_period_sec = 0.1,
}

MAP_BUILDER.use_trajectory_builder_3d = false
MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 50
TRAJECTORY_BUILDER_2D.min_range = 7.5
TRAJECTORY_BUILDER_2D.max_range = 45.0

TRAJECTORY_BUILDER_3D.min_range = 7.5
TRAJECTORY_BUILDER_3D.max_range = 45.0

return options
