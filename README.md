## turtlebot3_navi_my for Ros2 humble r2  
  
  for [foxbot_core3_r2.ino](https://github.com/tosa-no-onchan/foxbot_core3)  
  ROS2 Gazebo Turtlebot3 、 自作 Turtlebot3(foxbot_core3_r2.ino) を、 /cmd_vel 、 navigation2 を使って  
  動かす事ができる、 c++ プログラム。  
  
#### 1. How to install  

    $ cd ~/colcon_ws/src    
    $ git clone https://github.com/tosa-no-onchan/turtlebot3_navi_my.git    
    $ cd ..    
    $ colcon build --symlink-install --parallel-workers 1 --packages-select turtlebot3_navi_my  

### 2. How to run  

    1) ROS2 Gazebo Turtlebot3 、 ROS2 Turtlebot3 、 自作 Turtlebot3(foxbot_core3_r2.ino) を起動しておく。  

#### 2.1 navigatin2 を使う。  

    $ ros2 launch turtlebot3_navi_my multi_goals4_nav2.launch.py [use_sim_time:=True]  

#### 2.2 /cmd_vel を使う。  

    $ ros2 launch turtlebot3_navi_my multi_goals4_cmd_vel.launch.py [use_sim_time:=True]  
  
#### 3. コースを設定。  

    1) /cmd_vel  
    multi_goals4_cmd_vel.cpp の GoalList goallist[] で、指定する。  

    2) navigation2  
    mulit_goals4_nav2.cpp の GoalList goallist[] で、指定する。  

#### 4. update  
2023.3.16  

    1) foxbot_core3_r2 の herat beat に対応しました。  
    
2024.2.26  

    1) humble に対応しました。  

2024.3.12  

    1) Version r2 にしました。  
    Gazebo Turtlebot3 , Rtabmap_ros with RGBD Camera and Navigation2 で、AutoMapI AutoMapII が芳しく無いので、修正しました。

    
