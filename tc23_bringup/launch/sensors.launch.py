#!/usr/bin/env

import os

import yaml

from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros


def generate_launch_description():
    velodyne_pointcloud_share_dir = get_package_share_directory('velodyne_pointcloud')
    velodyne_transform_file = os.path.join(
        velodyne_pointcloud_share_dir,
        'config',
        'VLP32C-velodyne_transform_node-params.yaml'
    )
    with open(velodyne_transform_file, 'r') as f:
        converted_velodyne_transform_param = yaml.safe_load(f)['velodyne_transform_node']['ros__parameters']
    converted_velodyne_transform_param['calibration'] = os.path.join(
        velodyne_pointcloud_share_dir,
        'params',
        'VeloView-VLP-32C.yaml'
    )
    return launch.LaunchDescription([
        launch.actions.GroupAction([
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.AnyLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('realsense2_camera'),
                        'launch',
                        'rs_launch.py'
                    )
                ),
                launch_arguments={
                    'camera_name': 'd455',
                    'enable_depth': 'true',
                    'enable_infra': 'true',
                    'enable_infra1': 'true',
                    'enable_infra2': 'true',
                    'depth_module.exposure': '8500',
                    'depth_module.gain': '16',
                    'depth_module.hdr_enabled': 'false',
                    'enable_sync': 'true',
                    'enable_rgbd': 'true',
                    'enable_gyro': 'true',
                    'enable_accel': 'true',
                    'unite_imu_method': '1',
                    'align_depth.enable': 'true',
                    'decimation_filter.enable': 'true',
                    'spatial_filter.enable': 'true',
                    'temporal_filter.enable': 'true',
                    'hdr_merge.enable': 'false'
                }.items()
            ),
            launch_ros.actions.Node(
                package='velodyne_driver',
                executable='velodyne_driver_node',
                output='screen',
                parameters=[
                    os.path.join(
                        get_package_share_directory('velodyne_driver'),
                        'config',
                        'VLP32C-velodyne_driver_node-params.yaml'
                    ),
                    {'device_ip': '192.168.123.201'}
                ]
            ),
            launch_ros.actions.Node(
                package='velodyne_pointcloud',
                executable='velodyne_transform_node',
                output='screen',
                parameters=[converted_velodyne_transform_param]
            ),
            launch_ros.actions.Node(
                package='velodyne_laserscan',
                executable='velodyne_laserscan_node',
                output='screen',
                parameters=[
                    os.path.join(
                        get_package_share_directory('velodyne_laserscan'),
                        'config',
                        'default-velodyne_laserscan_node-params.yaml'
                    )
                ]
            )
        ])
    ])
