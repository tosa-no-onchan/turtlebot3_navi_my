/*
multi_goals4_cmd_vel.cpp
https://wiki.ros.org/pr2_controllers/Tutorials/Using%20the%20base%20controller%20with%20odometry%20and%20transform%20information

https://www.k-cube.co.jp/wakaba/server/func/math_h.html

https://answers.ros.org/question/50113/transform-quaternion/

build
$ colcon build --symlink-install --parallel-workers 1 --packages-select turtlebot3_navi_my
$ source install/local_setup.bash

#$ ros2 run turtlebot3_navi_my multi_goals4_cmd_vel
$ ros2 launch turtlebot3_navi_my multi_goals4_cmd_vel.launch.py use_sim_time:=[True|False]
*/

#include <memory>
#include "rclcpp/rclcpp.hpp"


#include <iostream>

//#include <ros/ros.h>
//#include <ros/topic.h>
//#include <geometry_msgs/Twist.h>
//#include <tf/transform_listener.h>
//#include "std_srvs/Empty.h"
#include <unistd.h>

//#include <math.h>

#include "turtlebot3_navi_my/pro_control.hpp"

//#include <nav_msgs/OccupancyGrid.h>
//from nav_msgs.msg import OccupancyGrid,Odometry

// https://progsennin.com/c-initstructarray/221/


std::string map_frame="map";
bool use_sim_time = false;

GoalList abnormal_test[] ={
            {60, 0.0, 0.0, 0.0},      // course correct ON
            {64, 0.0, 0.0, 0.0},      // go curve ON
            {66, 0.0, 0.0, 0.0},      // force current position to map(0,0)
            {67, 0.0, 0.0, 0.0},      // set dumper ON
            {0, 0.0, 0.0, 0.0},   // go (0.0,0.0) and rotate 0

            {0, 0.0, 0.0, -90.0},   // go (0.0,0.0) and rotate 90
            {0, 0.0, -0.5, -90.0},   // go (0.0,-0.5) and rotate 90
            {0, 0.0, -1.5, -90.0},   // go (0.0,-1.0) and rotate 90

            {99,0.0,0.0, 0.0}       // end
            };

GoalList goallist_test[] ={
            {60, 0.0, 0.0, 0.0},      // course correct ON
            {64, 0.0, 0.0, 0.0},      // go curve ON
            {66, 0.0, 0.0, 0.0},      // force current position to map(0,0)
            {67, 0.0, 0.0, 0.0},      // set dumper ON
            {0, 0.0, 0.0, 0.0},   // go (0.0,0.0) and rotate 0

            //{3, 0.0, 0.0, 90.0},    // rotate 90
            //{3, 0.0, 0.0, 90.0},   // rotate 90
            //{3, 0.0, 0.0, 90.0},   // rotate 90
            //{3, 0.0, 0.0, 90.0},   // rotate 90
            //{3, 0.0, 0.0, 360.0},   // rotate 360
            //{3, 0.0, 0.0, -360.0},   // rotate -360
            //{3, 0.0, 0.0, 200.0},   // rotate 200
            //{3, 0.0, 0.0, -200.0},   // rotate -200
            //{3, 0.0, 0.0, 20.0},   // rotate 20
            //{3, 0.0, 0.0, -20.0},   // rotate -20
            //{3, 0.0, 0.0, 30.0},   // rotate 30
            //{3, 0.0, 0.0, -30.0},   // rotate -30

            // 単体でのテスト
            //{0, 0.0, 0.0, 90.0},   // go (0.0,0.0) and rotate 90
            //{0, 0.0, 0.0, -90.0},   // go (0.0,0.0) and rotate 90
            //{0, 0.0, 0.0, 180.0},   // go (0.0,0.0) and rotate 180
            //{0, 0.0, 0.0, -180.0},   // go (0.0,0.0) and rotate -180

            // rotae_abs() の連続テスト  -> OK
            {0, 0.0, 0.0, 90.0},   // go (0.0,0.0) and rotate 90
            {0, 0.0, 0.0, 180.0},   // go (0.0,0.0) and rotate 90
            {0, 0.0, 0.0, 270.0},   // go (0.0,0.0) and rotate 90
            {0, 0.0, 0.0, 360.0},   // go (0.0,0.0) and rotate 90

            // rotae_abs() の連続テスト -> OK
            //{0, 0.0, 0.0, -90.0},   // go (0.0,0.0) and rotate -90
            //{0, 0.0, 0.0, -180.0},   // go (0.0,0.0) and rotate -180
            //{0, 0.0, 0.0, -270.0},   // go (0.0,0.0) and rotate -270
            //{0, 0.0, 0.0, -360.0},   // go (0.0,0.0) and rotate -360 

            // rotae_abs() 1回転 テスト -> 対応しない
            //{0, 0.0, 0.0, 360.0},   // go (0.0,0.0) and rotate 360  -> 対応しない
            //{0, 0.0, 0.0, -360.0},   // go (0.0,0.0) and rotate -360   -> 対応しない

            // rotae_abs() 1/2 pi, pi を跨った テスト
            //{0, 0.0, 0.0, 90.0+45.0},   // go (0.0,0.0) and rotate 135 -> OK
            //{0, 0.0, 0.0, 180.0+45.0},   // go (0.0,0.0) and rotate 225 -> OK

            //{0, 0.0, 0.0, 170.0},   // go (0.0,0.0) and rotate 170    -> OK
            //{0, 0.0, 0.0, 180.0+45.0},   // go (0.0,0.0) and rotate 225  -> OK

            //{0, 0.0, 0.0, 180.0+10.0},   // go (0.0,0.0) and rotate 190  -> OK

            // 反転 Zero のテスト  -> OK
            //{0, 0.0, 0.0, 90.0},   // go (0.0,0.0) and rotate 90
            //{0, 0.0, 0.0, 0.0},   // go (0.0,0.0) and rotate 0
            //{0, 0.0, 0.0, -90.0},   // go (0.0,0.0) and rotate -90
            //{0, 0.0, 0.0, 0.0},   // go (0.0,0.0) and rotate 0

            // 180 反転 のテスト -> OK
            //{0, 0.0, 0.0, 90.0},   // go (0.0,0.0) and rotate 90
            //{0, 0.0, 0.0, -90.0},   // go (0.0,0.0) and rotate -90  -> OK
            //{0, 0.0, 0.0, -90.0},   // go (0.0,0.0) and rotate -90
            //{0, 0.0, 0.0, 90.0},   // go (0.0,0.0) and rotate 90  -> OK


            // 180 反転 のテスト2 -> OK
            //{0, 0.0, 0.0, 90.0},   // go (0.0,0.0) and rotate 90
            //{0, 0.0, 0.0, 270.0},   // go (0.0,0.0) and rotate 270  -> OK
            //{0, 0.0, 0.0, -90.0},   // go (0.0,0.0) and rotate -90
            //{0, 0.0, 0.0, -270.0},   // go (0.0,0.0) and rotate -270  -> OK

            // 180 のテスト -> OK
            //{0, 0.0, 0.0, 180.0},   // go (0.0,0.0) and rotate 180 -> OK
            //{0, 0.0, 0.0, -180.0},   // go (0.0,0.0) and rotate -180 -> OK  --> 非対応にする。


            // 180 の反転テスト
            //{0, 0.0, 0.0, 180.0},   // go (0.0,0.0) and rotate 180
            //{0, 0.0, 0.0, -180.0},   // go (0.0,0.0) and rotate -180  -> 右回転 360(-) した。--> 非対応にする。

            // 180 の反転テスト
            //{0, 0.0, 0.0, -180.0},   // go (0.0,0.0) and rotate -180
            //{0, 0.0, 0.0, 180.0},   // go (0.0,0.0) and rotate 180  -> 左回転 360(+) した。  --> 非対応にする。


            // 180 の反転テスト
            //{0, 0.0, 0.0, 180.0},   // go (0.0,0.0) and rotate 180
            //{0, 0.0, 0.0, 180.0},   // go (0.0,0.0) and rotate 180   -> 動かない

            // 180 の反転テスト
            //{0, 0.0, 0.0, -180.0},   // go (0.0,0.0) and rotate -180
            //{0, 0.0, 0.0, -180.0},   // go (0.0,0.0) and rotate -180   -> 右回転 360(-) した。上と動きが違った。ちょっとの角度差なら、小さい方に回転させるべきか?

            // 戻りテスト
            //{0, 0.0, 0.0, 90.0},   // go (0.0,0.0) and rotate 90
            //{0, 0.0, 0.0, 45.0},   // go (0.0,0.0) and rotate 45
            //{0, 0.0, 0.0, -90.0},   // go (0.0,0.0) and rotate -90
            //{0, 0.0, 0.0, -45.0},   // go (0.0,0.0) and rotate -45


            {99,0.0,0.0, 0.0}       // end
            };

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


GoalList turtlebot3_house[] ={
            {60, 0.0, 0.0, 0.0},      // course correct ON
            {64, 0.0, 0.0, 0.0},      // go curve ON
            {66, 0.0, 0.0, 0.0},      // force current position to map(0,0)
            //{67, 0.0, 0.0, 0.0},      // set dumper ON
            {0, 0.0, 0.0, 0.0},         // go (0.0,0.0) and rotate 0
            //{2, 0.0, 0.0, 90.0},        // rotate 90
            //{2, 0.0, 0.0, 180.0},       // rotate 180
            //{2, 0.0, 0.0, 270.0},       // rotate 270
            //{2, 0.0, 0.0, 360},         // rotate 360
            //{0,0.0, -1.5, 270.0},      // go (0.0,-1.5) and rotate 270

            //{50,0.0,0.0, 0.0},          // set Navigation mode
            //{22,0.0,0.0, 0.0},        // get map
            //{99,0.0,0.0, 0.0},          // end

            {1,1.0, 0.0, 0.0},      // go (1.0,0.0)
            //{69,0.0, 0.0, 0.0},     // save local cost map
            //{99,0.0,0.0, 0.0},      // end
            //{2,1.0,0.0, 90.0},     // rotate 90
            //{2,1.0,0.0, 180.0},    // rotate 180
            //{2,1.0,0.0, 270.0},    // rotate 270
            //{2,1.0,0.0, 360.0},    // rotate 360
            //{2,0.0,0.0, 0.0},

            {1,2.0,0.0, 0.0},      // go (2.0,0.0) 
            {2,2.0,0.0, 90.0},     // rotate 90

            {1,2.0,1.5, 90.0},     // go (2.0,2.0) and rotate 90
            {1,2.0,3.0, 90.0},     // go (2.0,4.0) and rotate 90

            {2,2.0,3.0, 180.0},    // rotate 180
            {1,0.0,3.5, 180.0},     // go (4.0,4.0) 

            {1,-1.0,3.5, 180.0},     // go (4.0,4.0) 
            {2,-1.0,3.5, 270.0},    // rotate 270
            {99,0.0,0.0, 0.0},

            {0,2.0,0.0, 270.0},    // go (2.0,0.0) and rotate 270
            {2,2.0,0.0, -180.0},   // rotate -180

            //{50,0.0,0.0, 0.0},     // set Navigation mode

            //{2,0.0,0.0, 0.0},
            {99,0.0,0.0, 0.0},

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
            {22,0.0,0.0, 0.0},
            {99,0.0,0.0, 0.0}       // end
            };



GoalList goallist2[] ={
            {60, 0.0, 0.0, 0.0},      // course correct ON
            {64, 0.0, 0.0, 0.0},      // go curve ON
            {0, 0.0, 0.0, 0.0},   // go (0.0,0.0) and rotate 0
            {0,2.0,0.5, 0.0},      // go (2.0,0.5) and rotate 0
            {0,5.0,0.0, 0.0},      // go (5.0,0.0) and rotate 0

            //{50,0.0,0.0, 0.0},      // set Navigation mode
            //{22,0.0,0.0, 0.0},      // get map
            {99,0.0,0.0, 0.0},      // end

            {0,1.0,0.0, 0.0},      // go (1.0,0.0) and rotate 0
            {2,1.0,0.0, 90.0},     // rotate 90
            {2,1.0,0.0, 180.0},    // rotate 180
            {2,1.0,0.0, 270.0},    // rotate 270
            {2,1.0,0.0, 360.0},    // rotate 360
            //{2,0.0,0.0, 0.0},

            {0,2.0,0.0, 0.0},      // go (2.0,0.0) and rotate 0
            {2,2.0,0.0, 90.0},     // rotate 90

            {0,2.0,0.4, 90.0},     // go (2.0,0.4) and rotate 90
            {2,2.0,0.4, 180.0},    // rotate 180
            {2,2.0,0.4, 270.0},    // rotate 270

            {0,2.0,0.0, 270.0},    // go (2.0,0.0) and rotate 270
            {2,2.0,0.0, -180.0},   // rotate -180

            {50,0.0,0.0, 0.0},     // set Navigation mode

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
            {22,0.0,0.0, 0.0},
            {99,0.0,0.0, 0.0}       // end
            };


GoalList navi_list1[] ={
            {60, 0.0, 0.0, 0.0},    // course correct ON

            {10, 1.0, 0.0, 0},      // navi_move 1,0,0
            
            {2,1.0,0.0, 90.0},      // rotate 90
            {0,1.0,0.4, 90.0},      // go (2.0,0.4) and rotate 90

            //{50,0.0,0.0, 0.0},      // set Navigation mode
            //{22,0.0,0.0, 0.0},      // get map
            {99,0.0,0.0, 0.0},      // end
            };


GoalList2 m_goalList[] ={
            {60,0.0, 0.0},      // course correct ON
            {0, 0.0, 0},       // [0,0]
            {0, 0.0, 90},
            {0, 0.0, 180},
            {0, 0.0, 270},
            {0, 0.0, 360},
            //[2,0.0,0.0, 0.0],

            {0, 1.0, 0},        // [1,0]
            {0, 0.0, 90},
            {0, 0.0, 180},
            {0, 0.0, 270},
            {0, 0.0, 360},
            //[2,0.0,0.0, 0.0],

            {0, 1.0, 0},        // [2,0]
            {0, 0.0, 90},       // [2,0]

            {0, 0.4, 90},       // [2,0.4]
            {0,0.0, 180},
            {0,0.0, 270 },

            {0,0.4,270},         // [2,0]
            {0,0.0, -180},

            {50,0.0, 0.0},

            {60,0.0, 0.0},      // course correct ON
            //[62,0.0, 0.0],      // after_correct_wait ON
            //[99,0.0,0.0, 0.0],

            //[50,0.0,0.0, 0.0],

            {0,1.0, 180},     // [1.5,0]
            {0,0.0, 270},
            {0,0.0, 360},
            {0,0.0, 90},
            {0,0.0, 180},
            //[2,0.0,0.0, 0.0],

            {0,1.0, 180},      // [0,0]
            {0,0.0, 270},
            {0,0.0, 360},
            {22,0.0, 0.0},
            {99,0.0, 0.0}
            };


GoalList2 m_goalList2[] ={
            {60, 0.0, 0.0},      // course correct ON
            {64, 0.0, 0.0},      // go curve ON
            {66, 0.0, 0.0},      // force current position to map(0,0)
            {67, 0.0, 0.0},      // set dumper ON
            {0, 0.0, 90},       // rotate left 90
            {0, 0.0, 45},        // rotate left 45
            //{0, 2.0, 30},         // move back [-0.5,45]
            //{0, 0.0, 45},       // rotate left 45
            //{0, -0.5, 0.0},     // move back [-0.5,0]
            //{2,0.0,0.0, 0.0},
            //{0, 1.0, 0},        // [2,0]
            {99, 0.0, 0}        // [3,0]
            };


GoalList2 m_goalList3[] ={{60, 0.0, 0.0},      // course correct ON
            {0, 0.0, 0},       // [0,0]
            {0, 0.0, 90},
            {0, 0.0, 180},
            {0, 0.0, 270},
            {0, 0.0, 360},
            //{2,0.0,0.0, 0.0},

            {0, 0.0, 90},        // [1,0]
            {0, 1.0, 90},
            {0, 0.0, 180},
            {0, 0.0, 270},

            {0, 2.0, 270},
            {0, 0.0, -180},
            {0, 0.0, -270},
            {0, 1.0, -270},

            //[2,0.0,0.0, 0.0],

            {0,0.0, -360},
            {99,0.0,0}
            };

// 1平米を動く 0.8 x 0.8
GoalList2 m_square_1[] ={{0, 0.0, 90},  // (0,0)
            {0, 0.4, 90},   // (0,0.4)
            {0, 0.0, -360},
            {0, 0.8, 0},     // (0.8,0.4)
            {0, 0.0, -90},
            {0, 0.2, -90},   // (0.8,0.2)
            {0, 0.0, -180},
            {0, 0.8, -180},  // (0.0,0.2)
            {0, 0.0, 270}, 
            {0, 0.2, 270},   // (0.0,0.0)
            //{0, 0.0, 0},   // 
            //{0, 0.8, 0},   //  (0.8,0.0)
            //{0, 0.0, -90},  
            //{0, 0.2, -90},   // (0.8,-0.2)
            //{0, 0.0, -180},
            //{0, 0.8, -180},  // (0.0,-0.2)
            //{0, 0.0, 270},   
            //{0, 0.2, 270},   // (0.0,-0.4)
            //{0, 0.0, 0},   
            //{0, 0.8, 0},   // (0.8,-0.4)
            {99, 0.0, 0}   // (0.0,0.0)
            };

GoalList2 m_rotate_1[] ={{0, 0, 0},
            {0, 0, 45},
            {0, 0, 90},  // (0,0)
            {0, 0, 180},
            {0, 0, 270},
            {0, 0, 360},
            {99, 0, 0}
            //[0, 1.0, -90],   // (0,0.4)
            };


GoalList2 m_rotate_2[] ={{0, 0, -0},
            {0, 0, -90},  // (0,0)
            {0, 0, -180},
            {0, 0, -270},
            {0, 0, -360},
            {99, 0, 0}
            };


int main(int argc, char** argv)
{
    using namespace std::chrono_literals;

    std::cout << "start " << std::endl;

    //init the ROS node
    rclcpp::init(argc, argv);
    //ros::init(argc, argv, "muliti_goals4");

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("muliti_goals4",rclcpp::NodeOptions{});

    //ros::NodeHandle nh;
    //ros::NodeHandle nh("~");

    //rosparam
    //map_frame = rospy.get_param('~map_frame', 'map' )
    //nh.getParam("~map_frame",   map_frame);
    //use_sim_time=rospy.get_param('/use_sim_time')
    //nh.getParam("/use_sim_time",   use_sim_time);


    //GetMap mp;
    //mp.init(nh);
    //mp.get();

    //RobotNavi navi;
    //navi.init(nh);

    ProControl mg_ex;
    //mg_ex.init(nh);
    //mg_ex.init(node);
    // use local cost map changed by nishi 2024.9.6
    mg_ex.init(node,true);

    //ros::Rate rate(1);   //  1[Hz]
    //rclcpp::WallRate rate(1);
    //for(int i=0;i<10;i++){
    //    ros::spinOnce();
    //    rate.sleep();
    //}

    //mg_ex.mloop_ex(abnormal_test);
    //mg_ex.mloop_ex(goallist_test);
    mg_ex.mloop_ex(goallist);
    //mg_ex.mloop_ex(turtlebot3_house);
    //mg_ex.mloop_ex(goallist2);

    //mg_ex.mloop_ex2(m_rotate_2);
    //mg_ex.mloop_ex2(m_goalList2);

    //mg_ex.mloop_ex(navi_list1);

    //unsigned char cost = mg_ex.drive.navi_.check_cost(-3.0,1.0);
    //std::cout <<"cost = "<< std::hex << (unsigned int)cost << std::endl;

    //ros::Rate rate(1);   //  1[Hz]
    rclcpp::WallRate loop(1);
    while(rclcpp::ok()){
        //rclcpp::spin(std::make_shared<RobotDrive>());
        //get_tf.get(2);
        std::cout << "loop" << std::endl;
        loop.sleep();
    }
    //std::exit(0);
    rclcpp::shutdown();
    return 0;
}
