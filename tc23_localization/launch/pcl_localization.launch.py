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
import lifecycle_msgs.msg


def generate_launch_description():
    this_pkg_share_dir = get_package_share_directory('tc23_localization')
    use_sim_time = {
        'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time')
    }
    pcl_localization = launch_ros.actions.LifecycleNode(
        package='pcl_localization_ros2',
        executable='pcl_localization_node',
        name='pcl_localization',
        output='screen',
        namespace='',
        remappings=[
            ('/cloud', launch.substitutions.LaunchConfiguration('input_pointcloud_topic')),
            ('/imu', launch.substitutions.LaunchConfiguration('input_imu_topic'))
        ],
        parameters=[
            launch.substitutions.LaunchConfiguration(
                'localization_config'
            ),
            {'map_path': launch.substitutions.LaunchConfiguration('map_path')},
            use_sim_time
        ]
    )
    return launch.LaunchDescription([
        launch.actions.GroupAction([
            launch.actions.DeclareLaunchArgument(
                'use_sim_time',
                default_value='false'
            ),
            launch.actions.DeclareLaunchArgument(
                'localization_config',
                default_value=os.path.join(
                    this_pkg_share_dir,
                    'config',
                    'pcl_localization.yaml'
                )
            ),
            launch.actions.DeclareLaunchArgument(
                'map_path',
                default_value=os.path.join(
                    this_pkg_share_dir,
                    'maps',
                    'tc23_start_area.pcd'
                )
            ),
            launch.actions.DeclareLaunchArgument(
                'input_pointcloud_topic',
                default_value='velodyne_points'
            ),
            launch.actions.DeclareLaunchArgument(
                'input_pointcloud_frame_id',
                default_value='velodyne'
            ),
            launch.actions.DeclareLaunchArgument(
                'input_imu_topic',
                default_value='imu_data'
            ),
            launch.actions.DeclareLaunchArgument(
                'input_imu_frame_id',
                default_value='imu'
            ),
            launch.actions.DeclareLaunchArgument(
                'enable_stf',
                default_value='true'
            ),
            launch_ros.actions.Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='odom_to_base_stf',
                output='screen',
                arguments=[
                    '--frame-id', 'odom',
                    '--child-frame-id', 'base_link',
                    '--x', '0',
                    '--y', '0',
                    '--z', '0',
                    '--roll', '0',
                    '--pitch', '0',
                    '--yaw', '0'
                ],
                condition=launch.conditions.IfCondition(
                    launch.substitutions.LaunchConfiguration('enable_stf')
                )
            ),
            launch_ros.actions.Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_to_lidar_stf',
                output='screen',
                arguments=[
                    '--frame-id', 'base_link',
                    '--child-frame-id',
                    launch.substitutions.LaunchConfiguration('input_pointcloud_frame_id'),
                    '--x', '0',
                    '--y', '0',
                    '--z', '0',
                    '--roll', '0',
                    '--pitch', '0',
                    '--yaw', '0'
                ],
                condition=launch.conditions.IfCondition(
                    launch.substitutions.LaunchConfiguration('enable_stf')
                )
            ),
            launch_ros.actions.Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='lidar_to_imu_stf',
                output='screen',
                arguments=[
                    '--frame-id',
                    launch.substitutions.LaunchConfiguration('input_pointcloud_frame_id'),
                    '--child-frame-id',
                    launch.substitutions.LaunchConfiguration('input_imu_frame_id'),
                    '--x', '0',
                    '--y', '0',
                    '--z', '0',
                    '--roll', '0',
                    '--pitch', '0',
                    '--yaw', '0'
                ],
                condition=launch.conditions.IfCondition(
                    launch.substitutions.LaunchConfiguration('enable_stf')
                )
            ),
            pcl_localization,
            launch.actions.RegisterEventHandler(
                launch_ros.event_handlers.OnStateTransition(
                    target_lifecycle_node=pcl_localization,
                    goal_state='unconfigured',
                    entities=[
                        launch.actions.EmitEvent(
                            event=launch_ros.events.lifecycle.ChangeState(
                                lifecycle_node_matcher=launch.events.matches_action(
                                    pcl_localization
                                ),
                                transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE
                            )
                        )
                    ]
                )
            ),
            launch.actions.RegisterEventHandler(
                launch_ros.event_handlers.OnStateTransition(
                    target_lifecycle_node=pcl_localization,
                    start_state='configuring',
                    goal_state='inactive',
                    entities=[
                        launch.actions.EmitEvent(
                            event=launch_ros.events.lifecycle.ChangeState(
                                lifecycle_node_matcher=launch.events.matches_action(
                                    pcl_localization
                                ),
                                transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE
                            )
                        )
                    ]
                )
            ),
            launch.actions.EmitEvent(
                event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(
                        pcl_localization
                    ),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE
                )
            )
        ])
    ])
