# -*- coding: utf-8 -*-
# turtlebot3_navi_my/launch/go_auto_mower.launch.py
#
# 1. build on SBC and PC
#  $ colcon build --symlink-install --parallel-workers 1 --packages-select turtlebot3_navi_my
#  $ . install/setup.bash
#
# 2. run
# $ ros2 launch turtlebot3_navi_my go_auto_mower.launch.py
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
        DeclareLaunchArgument('threshold',default_value='250.0', description='White threshold'),
        DeclareLaunchArgument('plann_test',default_value='false', description='Plann create test'),
        DeclareLaunchArgument('all_nav2',default_value='false', description='All Navigation2 cotroll'),

        Node(
            package='turtlebot3_navi_my',executable='go_auto_mower',output="screen",
            #name='uvc_camera_stereo',
            #remappings=[('string_topic', '/talker')],
            emulate_tty=True,
            parameters=[
                        {"use_sim_time": use_sim_time,
                         "get_map_func": get_map_func,
                         "threshold": LaunchConfiguration('threshold'),
                         "plann_test": LaunchConfiguration('plann_test'),
                         "all_nav2": LaunchConfiguration('all_nav2'),
                        }
            ]
        )
    ])
