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
        DeclareLaunchArgument('robo_radius',default_value='0.3', description='robot radius [M]'),
        DeclareLaunchArgument('cource_width',default_value='8', description='robot runs lines width [dot]'),
        DeclareLaunchArgument('safe_margin',default_value='5', description='safe_margin robot side [dot]'),
        DeclareLaunchArgument('safe_margin_dt',default_value='5', description='safe_margin_dt direction of travel [dot]'),
        DeclareLaunchArgument('min_path_width_n',default_value='2', description='minimum path width factor N[times]'),
        # 狭小エリア minumum factor  最小幅 = safe_margin_dt*2+robot raius*min_path_width_n
        DeclareLaunchArgument('r_lng',default_value='0.6', description='obstacle aroud robot check radius [M]'),
        DeclareLaunchArgument('move_l',default_value='0.12', description='robot escape distance [M]'),

        DeclareLaunchArgument('robo_radian_marker',default_value='0.2', description='robot obstacle check maker [M]'),
        DeclareLaunchArgument('obstacle_eye_start',default_value='0.05', description='obstacle check eye start distance [M]'),
        DeclareLaunchArgument('obstacle_eye_stop',default_value='0.05', description='obstacle check eye stop distance [M]'),
        DeclareLaunchArgument('obstacle_eye_range',default_value='0.4', description='obstacle check eye range distance [M]'),

        DeclareLaunchArgument('map_orient_fix',default_value='true', description='local costmap global_frame:map or base_footpront'),
        DeclareLaunchArgument('ml_data',default_value='false', description='ML data collection'),
        DeclareLaunchArgument('opp_on',default_value='false', description='Opp with Lstm ON'),


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
                         "robo_radius": LaunchConfiguration('robo_radius'),
                         "cource_width": LaunchConfiguration('cource_width'),
                         "safe_margin": LaunchConfiguration('safe_margin'),
                         "safe_margin_dt": LaunchConfiguration('safe_margin_dt'),
                         "min_path_width_n": LaunchConfiguration('min_path_width_n'),
                         "r_lng": LaunchConfiguration('r_lng'),
                         "move_l": LaunchConfiguration('move_l'),
                         "robo_radian_marker": LaunchConfiguration('robo_radian_marker'),

                         "obstacle_eye_start": LaunchConfiguration('obstacle_eye_start'),
                         "obstacle_eye_stop": LaunchConfiguration('obstacle_eye_stop'),
                         "obstacle_eye_range": LaunchConfiguration('obstacle_eye_range'),

                         "map_orient_fix": LaunchConfiguration('map_orient_fix'),
                         "ml_data": LaunchConfiguration('ml_data'),
                         "opp_on": LaunchConfiguration('opp_on'),
                        }
            ]
        )
    ])
