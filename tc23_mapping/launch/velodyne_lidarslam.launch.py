#!/usr/bin/env

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


def generate_launch_description():
    this_pkg_share_dir = get_package_share_directory('tc23_mapping')

    use_sim_time = {
        'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time')
    }
    mapping_config = launch.substitutions.LaunchConfiguration(
        'mapping_config'
    )
    input_pointcloud_topic = launch.substitutions.LaunchConfiguration(
        'input_pointcloud_topic'
    )
    enable_stf = launch.substitutions.LaunchConfiguration(
        'enable_stf'
    )
    return launch.LaunchDescription([
        launch.actions.GroupAction([
            launch.actions.DeclareLaunchArgument(
                'use_sim_time',
                default_value='false'
            ),
            launch.actions.DeclareLaunchArgument(
                'mapping_config',
                default_value=os.path.join(
                    this_pkg_share_dir,
                    'config',
                    'tsukuba_lidarslam.yaml'
                )
            ),
            launch.actions.DeclareLaunchArgument(
                'input_pointcloud_topic',
                default_value='velodyne_points'
            ),
            launch.actions.DeclareLaunchArgument(
                'enable_stf',
                default_value='true'
            ),
            launch.actions.DeclareLaunchArgument(
                'with_rviz',
                default_value='true'
            ),
            launch_ros.actions.Node(
                package='scanmatcher',
                executable='scanmatcher_node',
                name='scan_matcher',
                output='screen',
                parameters=[
                    mapping_config,
                    use_sim_time
                ],
                remappings=[
                    ('input_cloud', input_pointcloud_topic)
                ]
            ),
            launch_ros.actions.Node(
                package='graph_based_slam',
                executable='graph_based_slam_node',
                name='graph_based_slam',
                output='screen',
                parameters=[
                    mapping_config,
                    use_sim_time
                ]
            ),
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.AnyLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('velodyne_pointcloud'),
                        'launch',
                        'velodyne_transform_node-VLP32C-launch.py'
                    )
                )
            ),
            launch_ros.actions.Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_to_lidar_stf',
                output='screen',
                parameters=[
                    use_sim_time
                ],
                arguments=[
                    '--frame-id', 'base_link',
                    '--child-frame-id', 'velodyne',
                    '--x', '0.0',
                    '--y', '0.0',
                    '--z', '0.2',
                    '--roll', '0.0',
                    '--pitch', '0.0',
                    '--yaw', '0.0'
                ],
                condition=launch.conditions.IfCondition(enable_stf)
            )
        ])
    ])
