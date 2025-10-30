# -*- coding: utf-8 -*-
# turtlebot3_navi_my/launch/go_auto_map.launch.py
#
# 1. build on SBC and PC
#  $ colcon build --symlink-install --parallel-workers 1 --packages-select turtlebot3_navi_my
#  $ . install/setup.bash
#
# 2. run
# $ ros2 launch turtlebot3_navi_my go_auto_map.launch.py
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

        # ブロブの作成時の、1[dot]の大きさ。あまり大きいと、ブロブが出来ないので注意。
        DeclareLaunchArgument('line_w',default_value='5', description='set line_w'),    # ラインの幅 -> grid size [dot]  0.05[m] * 5 = 25[cm]
        # 障害物との距離の調整に使います。単位: size [dot]
        DeclareLaunchArgument('robo_r',default_value='4', description='set robo_r'),    # robot 半径  -> grid size [dot]  0.05[m] * 5 = 25[cm]

        # obstacle_escape() 用のパラメータ
        DeclareLaunchArgument('r_lng',default_value='0.6', description='set r_lng'),
        DeclareLaunchArgument('black_thresh',default_value='0', description='set black_thresh'),
        DeclareLaunchArgument('move_l',default_value='0.12', description='set move_l'),

        Node(
            package='turtlebot3_navi_my',executable='go_auto_map',output="screen",
            #name='uvc_camera_stereo',
            #remappings=[('string_topic', '/talker')],
            emulate_tty=True,
            parameters=[
                        {"use_sim_time": use_sim_time,
                         "get_map_func": get_map_func,
                         "line_w": LaunchConfiguration('line_w'),
                         "robo_r": LaunchConfiguration('robo_r'),
                         "r_lng": LaunchConfiguration('r_lng'),
                         "black_thresh": LaunchConfiguration('black_thresh'),    # black count の閾値 = 0
                         "move_l": LaunchConfiguration('move_l'),
                        }
            ]
        )
    ])
