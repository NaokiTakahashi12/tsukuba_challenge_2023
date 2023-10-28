#!/usr/bin/env python3

# Copyright (c) 2023 Naoki Takahashi
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import os

from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros
import yaml


def generate_launch_description():
    this_pkg_share_dir = get_package_share_directory('tc23_mapping')

    use_sim_time = {
        'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time')
    }
    input_pointcloud_topic = launch.substitutions.LaunchConfiguration(
        'input_pointcloud_topic'
    )
    robot_description_str = launch.substitutions.Command([
        'xacro ', launch.substitutions.LaunchConfiguration('urdf_file')
    ])
    robot_description = {
        'robot_description': robot_description_str
    }

    velodyne_pointcloud_share_dir = get_package_share_directory('velodyne_pointcloud')
    vlp32c_transform_config_file = os.path.join(
        velodyne_pointcloud_share_dir,
        'config',
        'VLP32C-velodyne_transform_node-params.yaml'
    )
    with open(vlp32c_transform_config_file, 'r') as f:
        vlp32c_transform_config = yaml.safe_load(f)['velodyne_transform_node']['ros__parameters']
    vlp32c_transform_config['calibration'] = os.path.join(
        velodyne_pointcloud_share_dir,
        'params',
        'VeloView-VLP-32C.yaml'
    )

    exit_event = launch.actions.EmitEvent(
        event=launch.events.Shutdown()
    )

    return launch.LaunchDescription([
            launch.actions.DeclareLaunchArgument(
                'use_sim_time',
                default_value='false'
            ),
            launch.actions.DeclareLaunchArgument(
                'mapping_config_dir',
                default_value=os.path.join(
                    this_pkg_share_dir,
                    'config',
                )
            ),
            launch.actions.DeclareLaunchArgument(
                'mapping_config_file',
                default_value=os.path.join(
                    'tsukuba_cartographer_3d.lua'
                )
            ),
            launch.actions.DeclareLaunchArgument(
                'input_pointcloud_topic',
                default_value='velodyne_points'
            ),
            launch.actions.DeclareLaunchArgument(
                'urdf_file',
                default_value=os.path.join(
                    this_pkg_share_dir,
                    'urdf',
                    '202310_mapping_kit.urdf'
                )
            ),
            launch.actions.DeclareLaunchArgument(
                'with_rviz',
                default_value='true'
            ),
            launch_ros.actions.Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output='screen',
                parameters=[
                    use_sim_time,
                    robot_description
                ],
                on_exit=exit_event
            ),
            launch_ros.actions.Node(
                package='cartographer_ros',
                executable='cartographer_node',
                output='screen',
                parameters=[
                    use_sim_time
                ],
                arguments=[
                    '-configuration_directory',
                    launch.substitutions.LaunchConfiguration('mapping_config_dir'),
                    '-configuration_basename',
                    launch.substitutions.LaunchConfiguration('mapping_config_file')
                ],
                condition=launch.conditions.IfCondition('true'),
                on_exit=exit_event
            ),
            launch_ros.actions.Node(
                package='velodyne_pointcloud',
                executable='velodyne_transform_node',
                output='screen',
                parameters=[
                    vlp32c_transform_config,
                    use_sim_time
                ],
                on_exit=exit_event
            ),
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.AnyLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('velodyne_laserscan'),
                        'launch',
                        'velodyne_laserscan_node-launch.py'
                    )
                )
            ),
            launch_ros.actions.Node(
                package='pcl_ros',
                executable='filter_passthrough_node',
                name='pcl_filter_passthrough',
                output='screen',
                parameters=[
                    use_sim_time,
                    {'approximate_sync': True},
                    {'filter_field_name': 'z'},
                    {'filter_limit_min': -50.0},
                    {'filter_limit_max': 50.0},
                ],
                remappings=[
                    ('input', input_pointcloud_topic),
                    ('output', 'points2')
                ],
                on_exit=exit_event
            ),
    ])
