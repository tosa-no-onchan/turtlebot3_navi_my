/*
mulit_goals4_nav2.cpp
(旧 multi_goals4_move_base.cpp )

https://robot.isc.chubu.ac.jp/?p=1261
https://qiita.com/nnn112358/items/d159204d565469f647bb
https://inomacreate.com/original-robot04/


1. build
$ colcon build --symlink-install --parallel-workers 1 --packages-select turtlebot3_navi_my
$ . install/setup.bash

2. run
$ ros2 run turtlebot3_navi_my multi_goals4_nav2
*/

#include <iostream>

//#include <ros/ros.h>
//#include <ros/topic.h>
//#include <geometry_msgs/Twist.h>
//#include <tf/transform_listener.h>
//#include "std_srvs/Empty.h"
#include <unistd.h>

//#include <math.h>


//#define USE_MOVE_BASE
#include "turtlebot3_navi_my/multi_goals.hpp"


//#include <nav_msgs/OccupancyGrid.h>

GoalList goallist[] ={
            //{60, 0.0, 0.0, 0.0},      // course correct ON
            //{64, 0.0, 0.0, 0.0},      // go curve ON
            {66, 0.0, 0.0, 0.0},      // force current position to map(0,0)
            //{67, 0.0, 0.0, 0.0},      // set dumper ON
            {0, 0.0, 0.0, 0.0},   // go (0.0,0.0) and rotate 0
            //{0, 0.0, 0.0, 90.0},   // go (0.0,0.0) and rotate 0
            //{0, 0.0, 0.0, 180.0},   // go (0.0,0.0) and rotate 0
            //{0, 0.0, 0.0, 270.0},   // go (0.0,0.0) and rotate 0
            //{0, 0.0, 0.0, 360.0},   // go (0.0,0.0) and rotate 0
            //{99,0.0,0.0, 0.0},      // end

            //{0, 0.0, 0.0, -90.0},   // go (0.0,0.0) and rotate 0
            //{0, 0.0, 0.0, -180.0},   // go (0.0,0.0) and rotate 0
            //{0, 0.0, 0.0, -270.0},   // go (0.0,0.0) and rotate 0
            //{0, 0.0, 0.0, -360.0},   // go (0.0,0.0) and rotate 0

            //{99,0.0,0.0, 0.0},      // end

            {2, 0.0, 0.0, 90.0},  // rotate_abs 90
            {2, 0.0, 0.0, 180.0},  // rotate_abs 180
            {2, 0.0, 0.0, 270.0},  // rotate_abs 270
            {2, 0.0, 0.0, 360},   // rotate_abs 360
            //{99,0.0,0.0, 0.0},      // end

            //{50,0.0,0.0, 0.0},      // set Navigation mode
            //{22,0.0,0.0, 0.0},      // get map
            //{99,0.0,0.0, 0.0},      // end

            {0,1.0,0.0, 0.0},      // go (1.0,0.0) and rotate 0
            {2,1.0,0.0, 90.0},     // rotate 90
            {2,1.0,0.0, 180.0},    // rotate 180
            {2,1.0,0.0, 270.0},    // rotate 270
            {2,1.0,0.0, 360.0},    // rotate 360
            //{99,0.0,0.0, 0.0},      // end

            {0,2.0,0.0, 0.0},
            {2,2.0,0.0, 90.0},     // rotate 90


            {0,2.0,0.5, 90.0},     // go (2.0,0.4) and rotate 90
            {2,2.0,0.5, 180.0},    // rotate 180
            {2,2.0,0.5, 270.0},    // rotate 270

            {0,2.0,0.0, 270.0},    // go (2.0,0.0) and rotate 270
            {2,2.0,0.0, -180.0},   // rotate -180

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
            //{22,0.0,0.0, 0.0},      // get map
            {99,0.0,0.0, 0.0}       // end
            };


#define USE_1LOOP

GoalList turtlebot3_house[] ={
            //{60, 0.0, 0.0, 0.0},      // course correct ON
            //{64, 0.0, 0.0, 0.0},      // go curve ON
            {66, 0.0, 0.0, 0.0},      // force current position to map(0,0)
            //{67, 0.0, 0.0, 0.0},      // set dumper ON
            {0, 0.0, 0.0, 0.0},   // go (0.0,0.0) and rotate 0
            #ifdef USE_1LOOP
            {3, 0.0, 0.0, 90.0},    // rotate 90
            {3, 0.0, 0.0, 180.0},   // rotate 180
            {3, 0.0, 0.0, 270.0},   // rotate 270
            {3, 0.0, 0.0, 360.0},   // rotate 360
            #endif
            //{99,0.0,0.0, 0.0},      // end

            {0,2.0,0.0, 0.0},      // go (2.0,0.0) and rotate 0
            //{2,2.0,0.0, 90.0},     // rotate 90
            #ifdef USE_1LOOP
            {3, 0.0, 0.0, 90.0},    // rotate 90
            {3, 0.0, 0.0, 180.0},   // rotate 180
            {3, 0.0, 0.0, 270.0},   // rotate 270
            {3, 0.0, 0.0, 360.0},   // rotate 360
            #endif


            {0,2.0,2.0, 90.0},      // go (2.0,3.0) and rotate 90
            //{2,2.0,3.0, 180.0},      // rotate 180
            {0,2.0,3.0, 90.0},      // go (2.0,3.0) and rotate 90

            {0,0.0,3.0, 180.0},      // go (0.0,3.0) and rotate 180
            {0,-1.0,3.0, 180.0},      // go (-1.0,3.0) and rotate 180

            {0,-2.0,2.5, 180.0},      // go (-2.0,2.5) and rotate 180
            {0,-3.0,2.5, 270.0},      // go (-3.0,2.5) and rotate 270
            {0,-3.0,1.5, 270.0},      // go (-3.0,2.5) and rotate 270
            {0,-3.2,0.5, 270.0},      // go (-3.0,2.5) and rotate 270
            {0,-3.3,-0.5, 270.0},      // go (-3.0,2.5) and rotate 270
            {0,-3.3,-1.5, 270.0},      // go (-3.0,2.5) and rotate 270
            {0,-3.4,-2.5, 270.0},      // go (-3.0,2.5) and rotate 270
            {0,-3.4,-3.5, 270.0},      // go (-3.0,2.5) and rotate 270
            {0,-3.4,-4.0, 270.0},      // go (-3.0,2.5) and rotate 270

            {99,0.0,0.0, 0.0},      // end

            //{22,0.0,0.0, 0.0},      // get map
            {99,0.0,0.0, 0.0}       // end
            };


GoalList turtlebot3_house2[] ={
            //{60, 0.0, 0.0, 0.0},      // course correct ON
            //{64, 0.0, 0.0, 0.0},      // go curve ON
            {66, 0.0, 0.0, 0.0},      // force current position to map(0,0)
            //{67, 0.0, 0.0, 0.0},      // set dumper ON
            {0, 0.0, 0.0, 0.0},   // go (0.0,0.0) and rotate 0
            //{0, 0.0, 0.0, 90.0},   // go (0.0,0.0) and rotate 0
            //{0, 0.0, 0.0, 180.0},   // go (0.0,0.0) and rotate 0
            //{0, 0.0, 0.0, 270.0},   // go (0.0,0.0) and rotate 0
            //{0, 0.0, 0.0, 360.0},   // go (0.0,0.0) and rotate 0
            //{99,0.0,0.0, 0.0},      // end
            #ifdef USE_1LOOP
            {3, 0.0, 0.0, 90.0},    // rotate 90
            {3, 0.0, 0.0, 90.0},   // rotate 180
            {3, 0.0, 0.0, 90.0},   // rotate 270
            {3, 0.0, 0.0, 90.0},   // rotate 360
            #endif

            {0,1.0,0.0, 0.0},      // go (2.0,0.0) and rotate 0

            #ifdef USE_1LOOP
            {3, 0.0, 0.0, 90.0},    // rotate 90
            {3, 0.0, 0.0, 90.0},   // rotate 180
            {3, 0.0, 0.0, 90.0},   // rotate 270
            {3, 0.0, 0.0, 90.0},   // rotate 360
            #endif


            {0,2.0,0.0, 0.0},      // go (2.0,0.0) and rotate 0
            //{2,2.0,0.0, 90.0},     // rotate 90

            #ifdef USE_1LOOP
            {3, 0.0, 0.0, 90.0},    // rotate 90
            {3, 0.0, 0.0, 90.0},   // rotate 180
            {3, 0.0, 0.0, 90.0},   // rotate 270
            {3, 0.0, 0.0, 90.0},   // rotate 360
            #endif


            {0,2.5,-0.6, 0.0},      // go (2.5,-0.5) and rotate 0
            {0,3.0,-0.6, 0.0},      // go (2.5,-0.5) and rotate 0
            {0,4.0,-0.6, 0.0},      // go (2.5,-0.5) and rotate 0
            {0,5.0,-0.6, 0.0},      // go (2.5,-0.5) and rotate 0
            {0,6.0,-0.5, 0.0},      // go (2.5,-0.5) and rotate 0
            {0,7.0,-0.5, 0.0},      // go (2.5,-0.5) and rotate 0
            {0,8.0,-0.5, 0.0},      // go (2.5,-0.5) and rotate 0
            {0,9.5,-0.5, 0.0},      // go (2.5,-0.5) and rotate 0
            {99,0.0,0.0, 0.0},      // end

            {0,0.0,3.0, 180.0},      // go (0.0,3.0) and rotate 180
            {0,-1.0,3.0, 180.0},      // go (-1.0,3.0) and rotate 180

            {0,-2.0,2.5, 180.0},      // go (-2.0,2.5) and rotate 180
            {0,-3.0,2.5, 270.0},      // go (-3.0,2.5) and rotate 270
            {0,-3.0,1.5, 270.0},      // go (-3.0,2.5) and rotate 270
            {0,-3.2,0.5, 270.0},      // go (-3.0,2.5) and rotate 270
            {0,-3.3,-0.5, 270.0},      // go (-3.0,2.5) and rotate 270
            {0,-3.3,-1.5, 270.0},      // go (-3.0,2.5) and rotate 270
            {0,-3.4,-2.5, 270.0},      // go (-3.0,2.5) and rotate 270
            {0,-3.4,-3.5, 270.0},      // go (-3.0,2.5) and rotate 270
            {0,-3.4,-4.0, 270.0},      // go (-3.0,2.5) and rotate 270

            {99,0.0,0.0, 0.0},      // end

            //{22,0.0,0.0, 0.0},      // get map
            {99,0.0,0.0, 0.0}       // end
            };


GoalList turtlebot3_auto_map[] ={
            //{60, 0.0, 0.0, 0.0},      // course correct ON
            //{64, 0.0, 0.0, 0.0},      // go curve ON
            {66, 0.0, 0.0, 0.0},      // force current position to map(0,0)
            //{67, 0.0, 0.0, 0.0},      // set dumper ON
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

GoalList turtlebot3_auto_map_achor[] ={
            //{60, 0.0, 0.0, 0.0},      // course correct ON
            //{64, 0.0, 0.0, 0.0},      // go curve ON
            {66, 0.0, 0.0, 0.0},      // force current position to map(0,0)
            //{67, 0.0, 0.0, 0.0},      // set dumper ON
            {0, 0.0, 0.0, 0.0},       // go (0.0,0.0) and rotate 0
            {2, 0.0, 0.0, 90.0},      // rotate 90
            {2, 0.0, 0.0, 180.0},     // rotate 180
            {2, 0.0, 0.0, 270.0},     // rotate 270
            {2, 0.0, 0.0, 0.0},       // rotate 360
            //{10,0.0,1.0,90.0},         // navi move
            //{10,0.0,-1.0,-90.0},         // navi move
            {31,0.0,0.0, 0.0},        // Auto map builder of anchor
            {99,0.0,0.0, 0.0},        // end
};

int main(int argc, char **argv){
    using namespace std::chrono_literals;
    rclcpp::WallRate loop(1);

    std::cout << "start " << std::endl;

    //ros::init(argc, argv, "set_goal");
    //ros::init(argc, argv, "muliti_goals4_move_base");
    rclcpp::init(argc, argv);

    //ros::NodeHandle nh;
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("muliti_goals4");

    loop.sleep();


    MultiGoals mg_ex;
    //mg_ex.init(nh);
    mg_ex.init(node);


    mg_ex.mloop_ex(goallist);
    //mg_ex.mloop_ex(turtlebot3_house);
    //mg_ex.mloop_ex(turtlebot3_house2);
    //mg_ex.mloop_ex(turtlebot3_auto_map);
    //mg_ex.mloop_ex(turtlebot3_auto_map_achor);


    //ros::Rate rate(1);   //  1[Hz]
    while(rclcpp::ok()){
        //rclcpp::spin(std::make_shared<RobotDrive>());
        //get_tf.get(2);
        std::cout << "loop" << std::endl;
        loop.sleep();
    }
    //ros::spinOnce();

    //ros::spin();
    rclcpp::shutdown();

    return 0;
}
