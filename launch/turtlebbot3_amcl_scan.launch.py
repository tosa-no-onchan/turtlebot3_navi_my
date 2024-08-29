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
# this file
#  turtlebot3_navi_my/launch/turtlebbot3_amcl_scan.launch.py
#
#-------------------------------------------------------------
# 1. build
#  $ colcon build --symlink-install --parallel-workers 1 --packages-select turtlebot3_navi_my
#  $ . install/setup.bash
#
# Example:
#  1. Gazebo
#   $ export TURTLEBOT3_MODEL=waffle
#   $ . /usr/share/gazebo/setup.sh
#   $ ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
#   how to kill Gazeb server
#   $ killall gzserver
#
#  2. amcl scan [start slam,localization and navigation]
#  2.1 navigation [start localization and navigation ]
#   $ ros2 launch turtlebot3_navi_my turtlebbot3_amcl_scan.launch.py use_sim_time:=True [map:=FULL PATH] [params_file:=FULL PATH]
#
#  2.2 navigation and slam  [start slam and navigation]
#   $ ros2 launch turtlebot3_navi_my turtlebbot3_amcl_scan.launch.py use_sim_time:=True slam:=True [map:=FULL PATH] [params_file:=FULL PATH]
#
#  3.1 Rviz
#   $ ros2 launch nav2_bringup rviz_launch.py
#
#  4. C++ Program controll
#   $ ros2 launch turtlebot3_navi_my multi_goals4_cmd_vel.launch.py use_sim_time:=True
#   $ ros2 launch turtlebot3_navi_my multi_goals4_nav2.launch.py use_sim_time:=True
#
#  5. C++ Auto Map [navigation and slam]
#   $ ros2 launch turtlebot3_navi_my go_auto_map.launch.py use_sim_time:=True
#
#  6. C++ Auto Mower [localization and  navigation]
#   $ ros2 launch turtlebot3_navi_my go_auto_mower.launch.py use_sim_time:=True
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
        #default=os.path.join(
        #    get_package_share_directory('turtlebot3_navigation2'),
        #    'map',
        #    'map.yaml')
        #default=os.path.join(
        #    get_package_share_directory('turtlebot3_navi_my'),
        #    'map',
        #    'house_map.yaml')
        default=os.path.join('/','home','nishi','map','house_map.yaml'),
        ),

    #param_file_name = TURTLEBOT3_MODEL + '.yaml'
    param_dir = LaunchConfiguration(
        'params_file',
        #default=os.path.join(
        #    get_package_share_directory('turtlebot3_navigation2'),
        #    'param',
        #    param_file_name)
        default=os.path.join(get_package_share_directory('turtlebot3_navi_my'), 'params','amcl_scan', 'rpp_nav2_params.yaml'),
        ),

    slam = LaunchConfiguration('slam', default='False')


    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    rviz_config_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'slam',
            default_value='False',
            description='Use slam if true'),


        DeclareLaunchArgument('rviz',default_value='false', description='Launch RVIZ (optional).'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            # exec 3 launch.py
            # 1) slam_launch.py
            # 2) localization_launch.py
            # 3) navigation_launch.py
            # if you want reduce to exec launch, then set param for bringup_launch.py
            launch_arguments={
                'slam': slam,
                #'use_composition': 'True',
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
        ),

        Node(
            condition=IfCondition(LaunchConfiguration("rviz")),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
