## turtlebot3_navi_my for Ros2 humble r2  
  
  for [foxbot_core3_r2.ino](https://github.com/tosa-no-onchan/foxbot_core3)  
  ROS2 Gazebo Turtlebot3 、 自作 Turtlebot3(foxbot_core3_r2.ino) を、 /cmd_vel 、 navigation2 を使って  
  動かす事ができる、 c++ プログラム。  

  特に、Gazebo House , Turtlebot2 and Rtabmap_ros with RGBD Camera 環境があれば、簡単に下記テストができる。  
  #### Auto Map :  
  
    Rtabmap_ros with RGBD Camera で、部屋の中を勝手に動き回って、Map を作成する。  
    床の未走査エッジを自動検索して、動き回って、マップを作成します。  

説明は、下記ページにあります。  
[自作 Turtlebot3 自律走行に向けたプログラム。#11 AutoMap](http://www.netosa.com/blog/2022/07/-turtlebot3-11.html)  
    
  #### Auto Map II :  
  
    Rtabmap_ros with RGBD Camera で、部屋の中を勝手に動き回って、Map を作成する。  
    障害物のエッジを自動で動き回って、マップを作成します。  

説明は、下記ページにあります。  
[自作 Turtlebot3 自律走行に向けたプログラム。#12 AutoMap II](http://www.netosa.com/blog/2022/08/-turtlebot3-12.html)  
    
実行は、下記ランチファイルの上部の説明を参照してください。  
[turtlebot3_rgbd_sync.launch.py](https://github.com/tosa-no-onchan/rtabmap_ros_my/blob/main/launch/turtlebot3_rgbd_sync.launch.py)  
[turtlebot3_rgbd.launch.py](https://github.com/tosa-no-onchan/rtabmap_ros_my/blob/main/launch/turtlebot3_rgbd.launch.py)

  
#### 1. How to install  

    $ cd ~/colcon_ws/src    
    $ git clone https://github.com/tosa-no-onchan/turtlebot3_navi_my.git    
    $ cd ..    
    $ colcon build --symlink-install --parallel-workers 1 --packages-select turtlebot3_navi_my  

### 2. How to run  

    1) ROS2 Gazebo Turtlebot3 、 ROS2 Turtlebot3 、 自作 Turtlebot3(foxbot_core3_r2.ino) を起動しておく。  

  launch ファイル [tosa-no-onchan/rtabmap_ros_my](https://github.com/tosa-no-onchan/rtabmap_ros_my)  

#### 2.1 navigatin2 を使う。  

    $ ros2 launch turtlebot3_navi_my multi_goals4_nav2.launch.py [use_sim_time:=True]  

#### 2.2 /cmd_vel を使う。  

    $ ros2 launch turtlebot3_navi_my multi_goals4_cmd_vel.launch.py [use_sim_time:=True]  

#### 2.3 Auto Map を実行する。  

    $ ros2 launch turtlebot3_navi_my go_auto_map.launch.py [use_sim_time:=True] 
  
#### 3. コースを設定。  

    1) /cmd_vel  
    multi_goals4_cmd_vel.cpp の GoalList goallist[] で、指定する。  

    2) navigation2  
    mulit_goals4_nav2.cpp の GoalList goallist[] で、指定する。  

    3) Auto Map  
    go_auto_map.cpp の GoalList goallist[] で、指定する。  
    Auto Map  
    mg_ex.mloop_ex(turtlebot3_auto_map);  
    Auto Map II  
    mg_ex.mloop_ex(turtlebot3_auto_map_achor);  

#### 4. YouTube に動画をアップしました。  

[Auto Map II デモ](https://youtu.be/7o0vceDqD84?si=6Tb7DEqfdw9tuNvq)   
[![alt設定](http://img.youtube.com/vi/7o0vceDqD84/0.jpg)](https://youtu.be/7o0vceDqD84?si=6Tb7DEqfdw9tuNvq)


#### 5. update  
2023.3.16  

    1) foxbot_core3_r2 の herat beat に対応しました。  
    
2024.2.26  

    1) humble に対応しました。  

2024.3.12  

    1) Version r2 にしました。  
    Gazebo Turtlebot3 , Rtabmap_ros with RGBD Camera and Navigation2 で、AutoMapI AutoMapII が芳しく無いので、修正しました。

    
