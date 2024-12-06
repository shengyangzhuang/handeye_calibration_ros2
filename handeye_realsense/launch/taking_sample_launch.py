"""
Author: Shengyang Zhuang
Date Created: 2024-09-07
Description: ROS 2 launch file for recording robot and marker data simultaneously

Copyright © 2024 Shengyang Zhuang. All rights reserved.
This script is part of the "Multi-Robot System Prototyping for Cooperative Control in Robot-Assisted Spine Surgery" project and is authored solely by Shengyang Zhuang.

Project Website: https://shengyangzhuang.github.io/mres_thesis/
Shengyang Zhuang Personal Website: https://shengyangzhuang.github.io/

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at:
    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

# Note: Please retain this header in derivative works.

BibTeX:
@mastersthesis{zhuang2024multirobot,
  author    = {Zhuang, Shengyang},
  title     = {Multi-Robot System Prototyping for Cooperative Control in Robot-Assisted Spine Surgery},
  school    = {Imperial College London},
  year      = {2024},
}
"""


from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='handeye_realsense',
            executable='robot',
            name='robot_state_estimation'
        ),
        Node(
            package='handeye_realsense',
            executable='aruco',
            name='aruco_estimation'
        ),
    ])