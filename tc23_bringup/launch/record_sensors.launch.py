#!/usr/bin/env

import os

import yaml

from ament_index_python.packages import get_package_share_directory
import launch


def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.GroupAction([
        ])
    ])
