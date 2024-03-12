# -*- coding: utf-8 -*-
# turtlebot3_navi_my/launch/multi_goals4_cmd_vel.launch.py
#
# 1. build on SBC and PC
#  $ colcon build --symlink-install --parallel-workers 1 --packages-select turtlebot3_navi_my
#  $ . install/setup.bash
#
# 2. run
# $ ros2 launch turtlebot3_navi_my multi_goals4_cmd_vel.launch.py use_sim_time:=[True|False]
#
import os

#from launch import LaunchDescription
from launch_ros.actions import Node


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    get_map_func= LaunchConfiguration('get_map_func')

    return LaunchDescription([

        DeclareLaunchArgument('use_sim_time',default_value='false', description='sim time optional'),
        DeclareLaunchArgument('get_map_func',default_value='0', description='get_map_func optional'),

        Node(
            package='turtlebot3_navi_my',executable='multi_goals4_cmd_vel',output="screen",
            #name='uvc_camera_stereo',
            #remappings=[('string_topic', '/talker')],
            emulate_tty=True,
            parameters=[
                        {"use_sim_time": use_sim_time,
                         "get_map_func": get_map_func
                        }
            ]
        )
    ])
