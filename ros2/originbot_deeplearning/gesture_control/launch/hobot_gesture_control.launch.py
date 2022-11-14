# Copyright (c) 2022，Horizon Robotics.
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

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory

def generate_launch_description():
    web_service_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('websocket'),
                'launch/hobot_websocket_service.launch.py'))
    )

    return LaunchDescription([
        web_service_launch_include,
        # 启动图片发布pkg
        Node(
            package='mipi_cam',
            executable='mipi_cam',
            output='screen',
            parameters=[
                {"camera_calibration_file_path": "/opt/tros/lib/mipi_cam/config/GC4663_calibration.yaml"},
                {"out_format": "nv12"},
                {"image_width": 960},
                {"image_height": 544},
                {"io_method": "shared_mem"},
                {"video_device": "GC4663"}
            ],
            arguments=['--ros-args', '--log-level', 'error']
        ),
        # 启动jpeg图片编码&发布pkg
        Node(
            package='hobot_codec',
            executable='hobot_codec_republish',
            output='screen',
            parameters=[
                {"channel": 1},
                {"in_mode": "shared_mem"},
                {"in_format": "nv12"},
                {"out_mode": "ros"},
                {"out_format": "jpeg"},
                {"sub_topic": "/hbmem_img"},
                {"pub_topic": "/image_jpeg"}
            ],
            arguments=['--ros-args', '--log-level', 'error']
        ),
        # 启动单目rgb人体、人头、人脸、人手框和人体关键点检测pkg
        Node(
            package='mono2d_body_detection',
            executable='mono2d_body_detection',
            output='screen',
            parameters=[
                {"ai_msg_pub_topic_name": "/hobot_mono2d_body_detection"}
            ],
            arguments=['--ros-args', '--log-level', 'error']
        ),
        # 启动人手关键点检测pkg
        Node(
            package='hand_lmk_detection',
            executable='hand_lmk_detection',
            output='screen',
            parameters=[
                {"ai_msg_sub_topic_name": "/hobot_mono2d_body_detection"},
                {"ai_msg_pub_topic_name": "/hobot_hand_lmk_detection"}
            ],
            arguments=['--ros-args', '--log-level', 'error']
        ),
        # 启动web展示pkg
        Node(
            package='websocket',
            executable='websocket',
            output='screen',
            parameters=[
                {"image_topic": "/image_jpeg"},
                {"image_type": "mjpeg"},
                {"smart_topic": "/hobot_hand_gesture_detection"}
            ],
            arguments=['--ros-args', '--log-level', 'error']
        ),
        # 启动手势识别pkg
        Node(
            package='hand_gesture_detection',
            executable='hand_gesture_detection',
            output='screen',
            parameters=[
                {"ai_msg_sub_topic_name": "/hobot_hand_lmk_detection"},
                {"ai_msg_pub_topic_name": "/hobot_hand_gesture_detection"}
            ],
            arguments=['--ros-args', '--log-level', 'error']
        ),
        # 启动手势交互pkg
        Node(
            package='gesture_control',
            executable='gesture_control',
            output='screen',
            parameters=[
                {"ai_msg_sub_topic_name": "/hobot_hand_gesture_detection"},
                {"twist_pub_topic_name": "/cmd_vel"},
                {"activate_wakeup_gesture": 0},
                {"track_serial_lost_num_thr": 100},
                {"move_step": 0.3},
                {"rotate_step": 0.4}
            ],
            arguments=['--ros-args', '--log-level', 'info']
        )
    ])
