## turtlebot3_navi_my for Ros2 jazzy r7  
  
  Ros2 C++ Programmable Robot Control Stack for [foxbot_core3_r2.ino](https://github.com/tosa-no-onchan/foxbot_core3)  and Turtlebot3  
  --- ROS2 C++ roboto control Stack or Library ---  

  ROS2 C++ 自立走行ロボット開発 Stack と Application Samples.  

  ROS2 Turtlebot3(実機 or Gazebo) 、 自作 Turtlebot3(実機 foxbot_core3_r2.ino) など、 /cmd_vel 、 navigation2 でコントロールできるロボットを  
  C++ の簡略コードから、簡単に動かす事ができる、 プログラム Stack、C++ ライブラリー。  
  
  例えば、  
  前進して、(x,y) へ進め  
  右に 90度 回れ  
  など、簡単に C++ からプログラミンできる。  
  イメージとしては、Scratch（スクラッチ） の C++ ソースコード版。ただし、if loop 等の制御コードは無い。  
  また、class RobotDriveNAV2, class RobotDriveCmd_Vel を使えば、C++ コードから、Diff Drive ロボットを簡単に動かせる。  

  特に、Gazebo House , Turtlebot3 and Rtabmap_ros with RGBD Camera 環境があれば、簡単に下記テストができる。  
  Turtlebot3 amcl and scan の launch を追加しました。    
  #### cmd_vel :
  
    C++ プログラムから、/cmd_vel 操作でロボットを動かせる。  

  #### Navi2:
  
    C++ プログラムから、navigation2 操作でロボットを動かせる。  

  
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

  #### Auto Mower  

     Auto Map , MapII で作成した map を使って、ロボットの開始位置の自由領域を、ロボットの幅で  
     コースを作成して、それを走行する。

説明は、下記ページにあります。  
[ROS2 自作 Turtlebot3 による 草刈りロボット開発。#3 Auto Mower](http://www.netosa.com/blog/2024/04/ros2-turtlebot3-3.html)  
    
実行は、下記ランチファイルの上部の説明を参照してください。  
[turtlebot3_rgbd_sync.launch.py](https://github.com/tosa-no-onchan/rtabmap_ros_my/blob/main/launch/turtlebot3_rgbd_sync.launch.py)  
[turtlebot3_rgbd.launch.py](https://github.com/tosa-no-onchan/rtabmap_ros_my/blob/main/launch/turtlebot3_rgbd.launch.py)  
Turtlebot3 amcl and scan の launch を追加しました。  
[launch/turtlebot3_amcl_scan.launch.py](https://github.com/tosa-no-onchan/turtlebot3_navi_my/blob/main/launch/turtlebbot3_amcl_scan.launch.py)  

  
#### 1. How to install  

    $ cd ~/colcon_ws-jazzy/src    
    $ git clone https://github.com/tosa-no-onchan/turtlebot3_navi_my.git    
    $ cd ..    
    $ colcon build --symlink-install --parallel-workers 1 --packages-select turtlebot3_navi_my  

### 2. How to run  

    1) ROS2 Gazebo Turtlebot3 、 ROS2 Turtlebot3 、 自作 Turtlebot3(foxbot_core3_r2.ino) を起動しておく。  

  launch ファイル  
  [launch/turtlebot3_amcl_scan.launch.py](https://github.com/tosa-no-onchan/rtabmap_ros_my/blob/main/launch/turtlebot3_rgbd_sync.launch.py)  -- Turtlebot3 original slam,localization and navigation launch  
  [tosa-no-onchan/rtabmap_ros_my](https://github.com/tosa-no-onchan/rtabmap_ros_my)  -- Rtabmap_ros Depth Camera launch  

  各 lanuch ファイルの上部にガイドがあるので、それを参考にしてください。  

#### 2.1 navigatin2 を使う。  

    $ ros2 launch turtlebot3_navi_my multi_goals4_nav2.launch.py [use_sim_time:=True]  

#### 2.2 /cmd_vel を使う。  

    $ ros2 launch turtlebot3_navi_my multi_goals4_cmd_vel.launch.py [use_sim_time:=True]  

#### 2.3 Auto Map を実行する。  

    $ ros2 launch turtlebot3_navi_my go_auto_map.launch.py [use_sim_time:=True] 

#### 2.4 Auto Mower を実行する。    

    $ ros2 launch turtlebot3_navi_my go_auto_mower.launch.py [use_sim_time:=True] 

#### 3. 各 C++プログラムのコースを設定する。  
/cmd_vel と、navigation2 は、GoalList xxxlist[] を書き換えれば、自由にロボットをうごかせるぞね。  

    1) /cmd_vel  
    src/multi_goals4_cmd_vel.cpp の GoalList goallist[] で、指定する。  
    テーブルの中味を変えれば、自由にロボットを動かせる。  
    こちらは、/cmd_vel を使って、ロボットを動かせる。  
    こちらは、/cmd_vel が使えれば、動かせる。  
```
GoalList goallist[] ={
            {60, 0.0, 0.0, 0.0},      // course correct ON
            {64, 0.0, 0.0, 0.0},      // go curve ON
            {66, 0.0, 0.0, 0.0},      // force current position to map(0,0)
            //{67, 0.0, 0.0, 0.0},      // set dumper ON use local cost map
            //{82, 0.0, 0.0, 0.0},      // disable error auto stop
            //{0, 0.0, 0.0, 0.0},   // go (0.0,0.0) and rotate 0
            {2, 0.0, 0.0, 90.0},  // rotate 90
            {2, 0.0, 0.0, 180.0},  // rotate 180
            {2, 0.0, 0.0, 270.0},  // rotate 270
            {2, 0.0, 0.0, 360},   // rotate 360

            //{50,0.0,0.0, 0.0},      // set Navigation mode
            //{22,0.0,0.0, 0.0},      // get map
            //{99,0.0,0.0, 0.0},      // end

            {0,1.0,0.0, 0.0},      // go (1.0,0.0) and rotate 0
            {2,1.0,0.0, 90.0},     // rotate 90
            {2,1.0,0.0, 180.0},    // rotate 180
            {2,1.0,0.0, 270.0},    // rotate 270
            {2,1.0,0.0, 360.0},    // rotate 360
            //{2,0.0,0.0, 0.0},
            //{99,0.0,0.0, 0.0},      // end

            {0,2.0,0.0, 0.0},      // go (2.0,0.0) and rotate 0
            {2,2.0,0.0, 90.0},     // rotate 90

            {0,2.0,0.4, 90.0},     // go (2.0,0.4) and rotate 90
            {2,2.0,0.4, 180.0},    // rotate 180
            {2,2.0,0.4, 270.0},    // rotate 270

            {0,2.0,0.0, 270.0},    // go (2.0,0.0) and rotate 270
            {2,2.0,0.0, -180.0},   // rotate -180
            //{99,0.0,0.0, 0.0},      // end

            //{50,0.0,0.0, 0.0},     // set Navigation mode

            //{2,0.0,0.0, 0.0},
            //{99,0.0,0.0, 0.0},

            //{50,0.0,0.0, 0.0},

            {0,1.0,0.0, -180.0 },  // go (1.0,0.0) and rotate -180
            {2,1.0,0.0, 270.0 },   // rotate 270
            {2,1.0,0.0, 360.0},    // rotate 360
            {2,1.0,0.0, 90.0},     // rotate 90
            {2,1.0,0.0, 180.0},    // rotate 180
            //{2,0.0,0.0, 0.0},

            {0,0.0,0.0, 180.0},    // (0.0,0.0) and rotate 180
            {2,0.0,0.0, 270.0},    // rotate 270
            {2,0.0,0.0, 360.0},    // rotate 360
            //{22,0.0,0.0, 0.0},
            {99,0.0,0.0, 0.0}       // end
            };
```

    2) navigation2  
    src/mulit_goals4_nav2.cpp の GoalList goallist[] で、指定する。  
    テーブルの中味を変えれば、自由にロボットを動かせる。  
    こちらは、ROS2 Navigation2 を使って、ロボットを動かせる。  
    なので、Navigation 用の launch が必要。  
```
GoalList goallist[] ={
            {66, 0.0, 0.0, 0.0},      // force current position to map(0,0)
            {0, 0.0, 0.0, 0.0},   // go (0.0,0.0) and rotate 0
            {2, 0.0, 0.0, 90.0},  // rotate_abs 90
            {22,0.0,0.0, 0.0},      // get map
            {2, 0.0, 0.0, 180.0},  // rotate_abs 180
            {2, 0.0, 0.0, 270.0},  // rotate_abs 270
            {2, 0.0, 0.0, 360},   // rotate_abs 360
            {0,1.0,0.0, 0.0},      // go (1.0,0.0) and rotate 0
            {2,1.0,0.0, 90.0},     // rotate 90
            {2,1.0,0.0, 180.0},    // rotate 180
            {2,1.0,0.0, 270.0},    // rotate 270
            {2,1.0,0.0, 360.0},    // rotate 360
            {0,2.0,0.0, 0.0},
            {2,2.0,0.0, 90.0},     // rotate 90
            {0,2.0,0.5, 90.0},     // go (2.0,0.4) and rotate 90
            {2,2.0,0.5, 180.0},    // rotate 180
            {2,2.0,0.5, 270.0},    // rotate 270
            {0,2.0,0.0, 270.0},    // go (2.0,0.0) and rotate 270
            {2,2.0,0.0, -180.0},   // rotate -180
            {0,1.0,0.0, -180.0 },  // go (1.0,0.0) and rotate -180
            {2,1.0,0.0, 270.0 },   // rotate 270
            {2,1.0,0.0, 360.0},    // rotate 360
            {2,1.0,0.0, 90.0},     // rotate 90
            {2,1.0,0.0, 180.0},    // rotate 180
            {0,0.0,0.0, 180.0},    // (0.0,0.0) and rotate 180
            {2,0.0,0.0, 270.0},    // rotate 270
            {2,0.0,0.0, 360.0},    // rotate 360
            {22,0.0,0.0, 0.0},      // get map
            {99,0.0,0.0, 0.0}       // end
            };
```


    3) Auto Map  
    go_auto_map.cpp の GoalList goallist[] で、指定する。  
    Auto Map  
    mg_ex.mloop_ex(turtlebot3_auto_map);  
    Auto Map II  
    mg_ex.mloop_ex(turtlebot3_auto_map_achor);  
    ROS2 Navigation2 と SLAM が必要。  
```
// Auto Map I ( nav2 and cmd_vel mode)
GoalList turtlebot3_auto_map[] ={
            {60, 0.0, 0.0, 0.0},      // course correct ON
            {64, 0.0, 0.0, 0.0},      // go curve ON
            {66, 0.0, 0.0, 0.0},      // force current position to map(0,0)
            //{67, 0.0, 0.0, 0.0},      // set dumper ON
            // 障害物からの距離の調整
            {73, 6.0, 0.0, 0.0},      // set set robo_r_     waffle 281 x 306[mm]    30.6/5 = 6.12 -> 7 / 2 -> 4
            {0, 0.0, 0.0, 0.0},       // go (0.0,0.0) and rotate 0
            {2, 0.0, 0.0, 90.0},      // rotate 90
            {2, 0.0, 0.0, 180.0},     // rotate 180
            {2, 0.0, 0.0, 270.0},     // rotate 270
            {2, 0.0, 0.0, 0.0},       // rotate 360
            //{10,0.0,1.0,90.0},         // navi move
            //{10,0.0,-1.0,-90.0},         // navi move
            {30,0.0,0.0, 0.0},        // Auto map builder
            {99,0.0,0.0, 0.0},        // end
};

// Auto map II ( nav2 and cmd_vel mode)
GoalList turtlebot3_auto_map_achor[] ={
            {60, 0.0, 0.0, 0.0},      // course correct ON
            {64, 0.0, 0.0, 0.0},      // go curve ON
            {67, 0.0, 0.0, 0.0},      // set dumper ON
            // 障害物からの距離の調整
            {73, 6.0, 0.0, 0.0},      // set set robo_r_     waffle 281 x 306[mm]    30.6/5 = 6.12 -> 7 / 2 -> 4
            {66, 0.0, 0.0, 0.0},      // force current position to map(0,0)
            {0, 0.0, 0.0, 0.0},       // go (0.0,0.0) and rotate 0
            {2, 0.0, 0.0, 90.0},      // rotate 90
            {2, 0.0, 0.0, 180.0},     // rotate 180
            {2, 0.0, 0.0, 270.0},     // rotate 270
            {2, 0.0, 0.0, 0.0},       // rotate 360
            {31,0.0,0.0, 0.0},        // Auto map builder of anchor
            {99,0.0,0.0, 0.0},        // end
};
```
{30,0.0,0.0, 0.0},        // Auto map builder  
と   
{31,0.0,0.0, 0.0},        // Auto map builder of anchor  
で、すべて行っている。
カスタマイズするのでれば、src/pro_control_map.cpp の方をいじる。  


    4) Auto Mower  
    src/go_auto_mower.cpp の GoalList turtlebot3_auto_mower[] で指定する。  
    こちらは、/cmd_vel と Navigation2 を適宜使って、ロボットを動かす。  
    ROS2 Navigation2 と Static Map が必要。  
    こちらは、{35,0.0,0.0, 0.0},        // Auto Mower  
    で、全て、 src/pro_control_mower.cpp で実行する。  
    カスタマイズするのであれば、src/pro_control_mower.cpp の方をいじる。  
```
// Auto Mower ( nav2 and cmd_vel mode)
GoalList turtlebot3_auto_mower[] ={
            {60, 0.0, 0.0, 0.0},      // course correct ON
            {64, 0.0, 0.0, 0.0},      // go curve ON
            {66, 0.0, 0.0, 0.0},      // force current position to map(0,0)
            //{67, 0.0, 0.0, 0.0},      // set dumper ON
            // 障害物からの距離の調整 AutoMap の時有効だが!!
            {73, 6.0, 0.0, 0.0},      // set set robo_r_     waffle 281 x 306[mm]    30.6/5 = 6.12 -> 7 / 2 -> 4
            {0, 0.0, 0.0, 0.0},       // go (0.0,0.0) and rotate 0
            {2, 0.0, 0.0, 90.0},      // rotate 90
            //{2, 0.0, 0.0, 180.0},     // rotate 180
            //{2, 0.0, 0.0, 270.0},     // rotate 270
            //{2, 0.0, 0.0, 0.0},       // rotate 360
            //{10,0.0,1.0,90.0},         // navi move
            //{10,0.0,-1.0,-90.0},         // navi move
            {35,0.0,0.0, 0.0},        // Auto Mower
            {99,0.0,0.0, 0.0},        // end
};
```

#### 4. YouTube に動画をアップしました。  

[Auto Map II デモ](https://www.youtube.com/watch?v=7o0vceDqD84)   
[![alt設定](http://img.youtube.com/vi/7o0vceDqD84/0.jpg)](https://www.youtube.com/watch?v=7o0vceDqD84)  

[Auto Mower デモ](https://www.youtube.com/watch?v=7kpALJ3WcjQ)  
[![alt設定](http://img.youtube.com/vi/7kpALJ3WcjQ/0.jpg)](https://www.youtube.com/watch?v=7kpALJ3WcjQ)  

#### 5. update  
2023.3.16  

    1) foxbot_core3_r2 の herat beat に対応しました。  
    
2024.2.26  

    1) humble に対応しました。  

2024.3.12  

    1) Version r2 にしました。  
    Gazebo Turtlebot3 , Rtabmap_ros with RGBD Camera and Navigation2 で、AutoMapI AutoMapII が芳しく無いので、修正しました。

2024.4.6  

    1) Auto Mower , Turtlebot3 amcl and scan の launch を追加しました。  
    launch/turtlebot3_amcl_scan.launch.py  

2024.11.13 Version r5  
AutoMower で、Obstacle path planner with Transfomer or Lstm 用の学習データを保存できるようにしました。  

    1) Support automower saves the obsatcle image files ,which are the part of Global cost map.  
    They are used for trained data of opp_with_lstm and opp_with_transformer_cpp.  

    i) Edit turtlebot3_navi_my/include/ml_planner.hpp  
    std::string data_path_="/home/nishi/colcon_ws/src/turtlebot3_navi_my/ml_data/image";  // chage for your pc path.  
    
    $ colcon build --symlink-install --parallel-workers 1 --packages-select turtlebot3_navi_my  
    $ . install/setup.bash  

    ii) run accroding to top memo of turtlebot3_navi_my/launch/turtlebbot3_amcl_scan.launch.py   
    $ ros2 launch turtlebot3_navi_my go_auto_mower.launch.py use_sim_time:=True ml_data:=True

2025.1.3 Version r6  
AutoMower で、Obstacle path planner with Lstm tflite を使えるようにしました。  
        
    i) run accroding to top memo of turtlebot3_navi_my/launch/turtlebbot3_amcl_scan.launch.py   
    $ export LD_LIBRARY_PATH=/home/nishi/usr/local/lib/tensorflow-2.16.2-lite-flex:$LD_LIBRARY_PATH
    $ ros2 launch turtlebot3_navi_my go_auto_mower.launch.py use_sim_time:=True opp_on:=True  

上記、Obstacle path planner with Lstm tflite を使うためには、下記ページを参考に関連ライブラリーの組み込みが必要です。  
  
[ROS2 自作 Turtlebot3 による 草刈りロボット開発。#8 Transformer で経路計画をする。](https://www.netosa.com/blog/2024/09/ros2-turtlebot3-8-thetastarplanner.html)  
[ROS2 自作 Turtlebot3 による 草刈りロボット開発。#9 LSTM で経路計画をする。](https://www.netosa.com/blog/2024/11/ros2-turtlebot3-9-lstm.html)  
[tosa-no-onchan/opp_with_lstm](https://github.com/tosa-no-onchan/opp_with_lstm)  
[tosa-no-onchan/opp_with_transformer_cpp](https://github.com/tosa-no-onchan/opp_with_transformer_cpp)  
[TensorFlow 2.16.2 Lite C++ library build.](https://www.netosa.com/blog/2024/12/tensorflow-2162-lite-c-library-build.html)  
[Opp TensorFlow 2.16.2 Lite C++ library build.](https://www.netosa.com/blog/2024/12/opp-tensorflow-2162-lite-c-library-build.html)  
[tosa-no-onchan/opp_tflite](https://github.com/tosa-no-onchan/opp_tflite)  

2025.2.25 ROS2 Jazzy 版に対応しました。  
ROS2 Jazzy + Turtlebot4 + Gazebo Ionic warehouse Simulation で、 AutoMower を走らせる。  
[ROS2 Jazzy + Turtlebot4 + Gazebo Ionic warehouse Simulation.](https://www.netosa.com/blog/2025/02/ros2-jazzy-turtlebot4-gazebo-ionic-warehouse-simulation.html)  

    term1  Gazebo , Navi and Rviz2 All in one.  
    $ sudo ufw disable  
    $ ros2 launch nav2_bringup tb4_simulation_launch.py headless:=False params_file:=/home/nishi/colcon_ws-jazzy/src/turtlebot3_navi_my/params/amcl_scan/rpp_nav2_params.yaml  

    term2 term2 Auto Mower  
    $ export LD_LIBRARY_PATH=/home/nishi/usr/local/lib/tensorflow-lite-flex:$LD_LIBRARY_PATH  
    $ ros2 launch turtlebot3_navi_my go_auto_mower.launch.py use_sim_time:=True cource_width:=18 [plann_test:=True] [ml_data:=True] [opp_on:=True]  

2025.10.30 version-jazzy-r7  

    1) /cmd_vel の メッセージタイプを変更しました。  
    geometry_msgs/msg/Twist -> geometry_msgs/msg/TwistStamped  

    2) launch/tugbot_amcl_scan.launch.py を追加しました。  

    3) Tugbot Gazebo Wearhouse で動作確認して src/pro_control_map.cpp を改善しました。  
    launch/tugbot_amcl_scan.launch.py で動作確認出来ます。  
````
launch/tugbot_amcl_scan.launch.py で、Auto Map を走らせる。  
run:
1. Gazebo Tugbot Wearhouse
  $ sudo ufw disable
  $ ros2 launch tugbot_gazebo_my warehouse.launch.py
 
2. amcl scan
 2.1
  SLAM:
  $ ros2 launch turtlebot3_navi_my tugbot_amcl_scan.launch.py SBC:=true use_sim_time:=true slam:=True

3. Rviz
  $ ros2 launch turtlebot3_navi_my tugbot_amcl_scan.launch.py PC:=true use_sim_time:=true

6. C++ Auto Map [navigation and slam]
   今は、前半分のSCAN だと、AMCLがポンコツで未対応なので、下記手順をしてください。
   i) Teleop keyboard で、ロボットを、1[M] 前に移動させる。移動が終わったら、Ctl+C で終わらせる。
      $ ros2 run turtlebot3_teleop teleop_keyboard
        Ctl+C で終了
   ii) 実際の Auto Map を起動させる。
      $ ros2 launch turtlebot3_navi_my tugbot_amcl_scan.launch.py AUTO_MAP:=true use_sim_time:=true
````
