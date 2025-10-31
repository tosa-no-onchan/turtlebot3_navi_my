# -*- coding: utf-8 -*-
# Copyright 2019 Open Source Robotics Foundation, Inc.
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
#
# Author: Darby Lim
#
# original file
#  turtlebot3/turtlebot3_navigation2/launch/navigation2.launch.py
#
#-------------------------------------------------------------
# ROS2 jazzy
# tugbot_amcl_scan.launch.py
# 1. build
#  $ colcon build --symlink-install --parallel-workers 1 --packages-select turtlebot3_navi_my
#  $ . install/setup.bash
#
# run:
#  1. Gazebo Tugbot Wearhouse
#   $ sudo ufw disable
#   $ ros2 launch tugbot_gazebo_my warehouse.launch.py
# 
#  2. amcl scan
#  2.1
#   SLAM:
#   $ ros2 launch turtlebot3_navi_my tugbot_amcl_scan.launch.py SBC:=true use_sim_time:=true slam:=True
#
#  2.2 navigation
#   $ ros2 launch turtlebot3_navi_my tugbot_amcl_scan.launch.py SBC:=true use_sim_time:=true slam:=False
#
#  3. Rviz
#   $ ros2 launch turtlebot3_navi_my tugbot_amcl_scan.launch.py PC:=true use_sim_time:=true
#
#  4 robot control #2  on SBC or Remote PC
#  1)  Teleop keyboard
#   $ ros2 run turtlebot3_teleop teleop_keyboard
#
#  5. C++ Program controll
#   $ ros2 launch turtlebot3_navi_my multi_goals4_cmd_vel.launch.py use_sim_time:=True
#   $ ros2 launch turtlebot3_navi_my multi_goals4_nav2.launch.py use_sim_time:=True
#
#  6. C++ Auto Map [navigation and slam]
#    今は、前半分のSCAN だと、AMCLがポンコツで未対応なので、下記手順をしてください。
#   i) Teleop keyboard で、ロボットを、1[M] 前に移動させる。移動が終わったら、Ctl+C で終わらせる。
#      $ ros2 run turtlebot3_teleop teleop_keyboard
#        Ctl+C で終了
#   ii) 実際の Auto Map を起動させる。
#      $ ros2 launch turtlebot3_navi_my tugbot_amcl_scan.launch.py AUTO_MAP:=true use_sim_time:=true
#
#  7. C++ Auto Mower [localization and  navigation]
#   $ export LD_LIBRARY_PATH=/home/nishi/usr/local/lib/tensorflow-lite-flex:$LD_LIBRARY_PATH
#   $ ros2 launch turtlebot3_navi_my go_auto_mower.launch.py use_sim_time:=True cource_width:=18 [plann_test:=True] [ml_data:=True] [opp_on:=True]
#
# append.
# how to map save
# ros2 run nav2_map_server map_saver_cli -f ~/map/house_map --ros-args -p save_map_timeout:=10000.0

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

#TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_dir = LaunchConfiguration(
        'map',
        #default=os.path.join(get_package_share_directory('turtlebot3_navigation2'),'map','map.yaml')
        #default=os.path.join(get_package_share_directory('turtlebot3_navi_my'),'map','house_map.yaml')
        default=os.path.join(get_package_share_directory('turtlebot3_navi_my'),'map','depot.yaml')
        #default=os.path.join('/','home','nishi','map','house_map.yaml'),
        #default=os.path.join('/','home','nishi','map','depot.yaml'),
        ),

    #param_file_name = TURTLEBOT3_MODEL + '.yaml'
    param_dir = LaunchConfiguration(
        'params_file',
        #default=os.path.join(
        #    get_package_share_directory('turtlebot3_navigation2'),
        #    'param',
        #    param_file_name)
        default=os.path.join(get_package_share_directory('turtlebot3_navi_my'), 'params', 'tugbot', 'rpp_nav2_params.yaml'),
        ),
    slam = LaunchConfiguration('slam')
    use_composition = LaunchConfiguration('use_composition')
    turtlebot3_navi_my=get_package_share_directory('turtlebot3_navi_my')
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    get_map_func= LaunchConfiguration('get_map_func')

    rviz_config_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('SBC', default_value='false', description='Launch SBC (optional).'),
        DeclareLaunchArgument('PC', default_value='false', description='Launch PC (optional).'),
        DeclareLaunchArgument('AUTO_MAP', default_value='false', description='Launch Auto Map C++.'),
        DeclareLaunchArgument('use_sim_time',default_value='false',description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('qos', default_value='1',description='QoS used for input sensor topics'),
        DeclareLaunchArgument('map',default_value=map_dir,description='Full path to map file to load'),
        DeclareLaunchArgument('params_file',default_value=param_dir,description='Full path to param file to load'),
        DeclareLaunchArgument('slam',default_value='False',description='Use slam if true'),
        DeclareLaunchArgument('use_composition',default_value='True',description='Use use_composition if true'),

        # go_auto_map.launch.py params
        DeclareLaunchArgument('get_map_func',default_value='0', description='get_map_func optional'),
        # ブロブの作成時の、1[dot]の大きさ。あまり大きいと、ブロブが出来ないので注意。
        DeclareLaunchArgument('line_w',default_value='5', description='set line_w'),    # ラインの幅 -> grid size [dot]  0.05[m] * 5 = 25[cm]
        # 障害物との距離の調整に使います。単位: size [dot]
        DeclareLaunchArgument('robo_r',default_value='8', description='set robo_r'),    # robot 半径  -> grid size [dot]  0.05[m] * 5 = 25[cm]
        # for obstacle_escape()
        # 狭小エリア minumum factor  最小幅 = safe_margin_dt*2+robot raius*min_path_width_n
        #DeclareLaunchArgument('r_lng',default_value='0.6', description='obstacle aroud robot check radius [M]'),
        #DeclareLaunchArgument('r_lng',default_value='0.65', description='obstacle aroud robot check radius [M]'),
        DeclareLaunchArgument('r_lng',default_value='0.7', description='obstacle aroud robot check radius [M]'),
        DeclareLaunchArgument('black_thresh',default_value='0', description='get_map_func optional'),
        #DeclareLaunchArgument('move_l',default_value='0.12', description='robot escape distance [M]'),
        DeclareLaunchArgument('move_l',default_value='0.17', description='robot escape distance [M]'),
        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource([merge_scaner_launch_file_dir, '/start_demo_tugbot.launch.py']),
        #),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            # exec 3 launch.py
            # 1) slam_launch.py
            # 2) localization_launch.py
            # 3) navigation_launch.py
            # if you want reduce to exec launch, then set param for bringup_launch.py
            launch_arguments={
                'slam': slam,
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'use_composition': use_composition,
                'enable_stamped_cmd_vel': 'True',
                #'use_composition': 'True',
                'params_file': param_dir}.items(),
            condition=IfCondition(LaunchConfiguration('SBC')),
        ),

        Node(
            condition=IfCondition(LaunchConfiguration('PC')),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(turtlebot3_navi_my , 'launch', 'go_auto_map.launch.py')
            ),
            launch_arguments={
                        "use_sim_time": use_sim_time,
                        "get_map_func": get_map_func,
                        "line_w": LaunchConfiguration('line_w'),
                        "robo_r": LaunchConfiguration('robo_r'),
                        "r_lng": LaunchConfiguration('r_lng'),
                        "black_thresh": LaunchConfiguration('black_thresh'),    # black count の閾値 = 0
                        "move_l": LaunchConfiguration('move_l'),
                        }.items(),
            condition=IfCondition(LaunchConfiguration('AUTO_MAP')),
        ),
    ])
