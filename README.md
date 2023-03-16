## turtlebot3_navi_my for Ros2 galactic  
  
  for [foxbot_core3_r2.ino](https://github.com/tosa-no-onchan/foxbot_core3)  
  ROS2 Turtlebot3 、 自作 Turtlebot3(foxbot_core3_r2.ino) を、 /cmd_vel 、 navigation2 を使って  
  動かす事ができる、 c++ プログラム。  
  
#### 1. How to install  

    $ cd ~/colcon_ws/src    
    $ git clone https://github.com/tosa-no-onchan/turtlebot3_navi_my.git    
    $ cd ..    
    $ colcon build --symlink-install --parallel-workers 1 --packages-select turtlebot3_navi_my  

### 2. How to run  

    1) ROS2 Gazebo Turtlebot3 、 ROS2 Turtlebot3 、 自作 Turtlebot3(foxbot_core3_r2.ino) を起動しておく。  

#### 2.1 navigatin2 を使う。  

    $ ros2 run turtlebot3_navi_my multi_goals4_nav2  

#### 2.2 /cmd_vel を使う。  

    $ ros2 run turtlebot3_navi_my multi_goals4_cmd_vel
  
#### 3. コースを設定。  

    1) /cmd_vel  
    multi_goals4_cmd_vel.cpp の GoalList goallist[] で、指定する。  

    2) navigation2  
    mulit_goals4_nav2.cpp の GoalList goallist[] で、指定する。  

#### 4. update  
2023.3.16  

    1) foxbot_core3_r2 の herat beat に対応しました。  
    
