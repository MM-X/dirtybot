#!/usr/bin/python3

# Copyright (c) 2022, www.guyuehome.com
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='use_lidar', default_value='false'),
        launch.actions.DeclareLaunchArgument(name='use_camera', default_value='false'),
        launch.actions.DeclareLaunchArgument(name='use_imu', default_value='false'),

        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('originbot_base'),
                             'launch/robot.launch.py')),
                launch_arguments={
                'use_imu': launch.substitutions.LaunchConfiguration('use_imu')}.items(), 
                ),

        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('originbot_bringup'),
                             'launch','ydlidar.launch.py')),
                condition=launch.conditions.IfCondition(
                    launch.substitutions.LaunchConfiguration('use_lidar'))),

        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('originbot_bringup'),
                             'launch','camera.launch.py')),
                condition=launch.conditions.IfCondition(
                    launch.substitutions.LaunchConfiguration('use_camera')))
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
