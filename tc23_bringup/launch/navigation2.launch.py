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
    # this_pkg_share_dir = get_package_share_directory('tc23_bringup')
    # this_pkg_include_launch_dir = os.path.join(
    #     this_pkg_share_dir,
    #     'launch',
    #     'includes'
    # )
    nav2_bringup_share_dir = get_package_share_directory('nav2_bringup')
    tc23_nav2_config_share_dir = get_package_share_directory('tc23_nav2_config')

    namespace = launch.substitutions.LaunchConfiguration('namespace')
    no_robot = launch.substitutions.LaunchConfiguration('no_robot')
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time')
    autostart = launch.substitutions.LaunchConfiguration('autostart')
    params_file = launch.substitutions.LaunchConfiguration('params_file')

    use_sim_time_param = {'use_sim_time': use_sim_time}
    autostart_param = {'autostart': autostart}

    no_robot_condition = launch.conditions.IfCondition(no_robot)

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'namespace',
            default_value=['']
        ),
        launch.actions.DeclareLaunchArgument(
            'use_sim_time',
            default_value=['false']
        ),
        launch.actions.DeclareLaunchArgument(
            'autostart',
            default_value=['true']
        ),
        launch.actions.DeclareLaunchArgument(
            'map_file',
            default_value=os.path.join(
                nav2_bringup_share_dir,
                'maps',
                'turtlebot3_world.yaml'
            )
        ),
        launch.actions.DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                tc23_nav2_config_share_dir,
                'config',
                'nav2.yaml'
            )
        ),
        launch.actions.DeclareLaunchArgument(
            'no_robot',
            default_value=['true']
        ),
        launch.actions.GroupAction([
            launch_ros.actions.Node(
                package='nav2_map_server',
                executable='map_server',
                output='screen',
                parameters=[
                    {'yaml_filename': launch.substitutions.LaunchConfiguration('map_file')},
                    use_sim_time_param
                ],
            ),
            launch_ros.actions.Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='map_to_odom_stf',
                output='screen',
                parameters=[
                    use_sim_time_param
                ],
                arguments=[
                    '--frame-id', 'map',
                    '--child-frame-id', 'odom',
                    '--x', '-1.8'
                ],
                condition=no_robot_condition
            ),
            launch_ros.actions.Node(
                package='dummy_odometry',
                executable='dummy_odometry_node',
                name='dummy_odometry',
                output='screen',
                remappings=[
                    ('~/cmd_vel', 'cmd_vel_nav'),
                    ('~/odom', 'odom')
                ],
                condition=no_robot_condition
            ),
            launch.actions.TimerAction(
                period=5.0,
                actions=[
                    launch_ros.actions.Node(
                        package='nav2_lifecycle_manager',
                        executable='lifecycle_manager',
                        name='lifecycle_manager_map_server',
                        output='screen',
                        parameters=[
                            autostart_param,
                            {'node_names': ['map_server']},
                            use_sim_time_param
                        ],
                    ),
                ]
            )
        ]),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.AnyLaunchDescriptionSource(
                os.path.join(
                    nav2_bringup_share_dir,
                    'launch',
                    'navigation_launch.py'
                )
            ),
            launch_arguments={
                'namespace': namespace,
                'autostart': autostart,
                'params_file': params_file,
                'use_lifecycle_mgr': 'true',
                'use_sim_time': use_sim_time
            }.items()
        )
    ])
