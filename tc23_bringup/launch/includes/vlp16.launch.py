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

import yaml
from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros


def generate_launch_description():
    velodyne_pointcloud_share_dir = get_package_share_directory('velodyne_pointcloud')
    velodyne_transform_file = os.path.join(
        velodyne_pointcloud_share_dir,
        'config',
        'VLP16-velodyne_transform_node-params.yaml'
    )
    with open(velodyne_transform_file, 'r') as f:
        converted_velodyne_transform_param \
            = yaml.safe_load(f)['velodyne_transform_node']['ros__parameters']
    converted_velodyne_transform_param['calibration'] = os.path.join(
        velodyne_pointcloud_share_dir,
        'params',
        'VLP16db.yaml'
    )
    return launch.LaunchDescription([
        launch.actions.GroupAction([
            launch_ros.actions.Node(
                package='velodyne_driver',
                executable='velodyne_driver_node',
                output='screen',
                parameters=[
                    os.path.join(
                        get_package_share_directory('velodyne_driver'),
                        'config',
                        'VLP16-velodyne_driver_node-params.yaml'
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
