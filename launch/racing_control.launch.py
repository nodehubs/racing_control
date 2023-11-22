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
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'avoid_angular_ratio',
            default_value='0.2',
            description='image subscribe topic name'),
        DeclareLaunchArgument(
            'avoid_linear_speed',
            default_value='0.1',
            description='image subscribe topic name'),
        DeclareLaunchArgument(
            'follow_angular_ratio',
            default_value='-1.0',
            description='image subscribe topic name'),
        DeclareLaunchArgument(
            'follow_linear_speed',
            default_value='0.1',
            description='image subscribe topic name'),
        DeclareLaunchArgument(
            'bottom_threshold',
            default_value='200',
            description='image subscribe topic name'),
        DeclareLaunchArgument(
            'confidence_threshold',
            default_value='0.5',
            description='image subscribe topic name'),
        Node(
            package='racing_control',
            executable='racing_control',
            output='screen',
            parameters=[
                {"pub_control_topic": "/cmd_vel"},
                {"avoid_angular_ratio": LaunchConfiguration('avoid_angular_ratio')},
                {"avoid_linear_speed": LaunchConfiguration('avoid_linear_speed')},
                {"follow_angular_ratio": LaunchConfiguration('follow_angular_ratio')},
                {"follow_linear_speed": LaunchConfiguration('follow_linear_speed')},
                {"bottom_threshold": LaunchConfiguration('bottom_threshold')},
                {"confidence_threshold": LaunchConfiguration('confidence_threshold')},
            ],
            arguments=['--ros-args', '--log-level', 'warn']
        )

    ])