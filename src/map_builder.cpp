/*
map_builder.cpp

https://wiki.ros.org/pr2_controllers/Tutorials/Using%20the%20base%20controller%20with%20odometry%20and%20transform%20information

https://www.k-cube.co.jp/wakaba/server/func/math_h.html

https://answers.ros.org/question/50113/transform-quaternion/

build
$ catkin build turtlebot3_navi_my

$ rosrun turtlebot3_navi_my map_builder
*/

#include <iostream>

#include <ros/ros.h>
#include <ros/topic.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include "std_srvs/Empty.h"
#include <unistd.h>

//#include <math.h>

#include "turtlebot3_navi_my/multi_goals.h"

#include <nav_msgs/OccupancyGrid.h>
//from nav_msgs.msg import OccupancyGrid,Odometry

// https://progsennin.com/c-initstructarray/221/


std::string map_frame="map";
bool use_sim_time = false;


GoalList turtlebot3_house[] ={
            {60, 0.0, 0.0, 0.0},      // course correct ON
            {64, 0.0, 0.0, 0.0},      // go curve ON
            {66, 0.0, 0.0, 0.0},      // force current position to map(0,0)
            {67, 0.0, 0.0, 0.0},      // set dumper ON
            {0, 0.0, 0.0, 0.0},       // go (0.0,0.0) and rotate 0
            {2, 0.0, 0.0, 90.0},      // rotate 90
            {2, 0.0, 0.0, 180.0},     // rotate 180
            {2, 0.0, 0.0, 270.0},     // rotate 270
            {2, 0.0, 0.0, 360},       // rotate 360
            {22,0.0,0.0, 0.0},        // get map
            {99,0.0,0.0, 0.0},        // end
            //{0,0.0, -1.5, 270.0},      // go (0.0,-1.5) and rotate 270

            //{50,0.0,0.0, 0.0},      // set Navigation mode
            //{22,0.0,0.0, 0.0},      // get map
            //{99,0.0,0.0, 0.0},      // end

            {0,1.0, 0.0, 0.0},      // go (1.0,0.0) and rotate 0
            //{69,0.0, 0.0, 0.0},     // save local cost map
            //{99,0.0,0.0, 0.0},      // end
            {2,1.0,0.0, 90.0},     // rotate 90
            {2,1.0,0.0, 180.0},    // rotate 180
            {2,1.0,0.0, 270.0},    // rotate 270
            {2,1.0,0.0, 360.0},    // rotate 360
            //{2,0.0,0.0, 0.0},

            {0,2.0,0.0, 0.0},      // go (2.0,0.0) and rotate 0
            {2,2.0,0.0, 90.0},     // rotate 90
            {22,0.0,0.0, 0.0},        // get map
            {99,0.0,0.0, 0.0},      // end



            {0,2.0,0.4, 90.0},     // go (2.0,0.4) and rotate 90
            {2,2.0,0.4, 180.0},    // rotate 180
            {2,2.0,0.4, 270.0},    // rotate 270

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


GoalList turtlebot3_auto_map[] ={
            {60, 0.0, 0.0, 0.0},      // course correct ON
            {64, 0.0, 0.0, 0.0},      // go curve ON
            {66, 0.0, 0.0, 0.0},      // force current position to map(0,0)
            {67, 0.0, 0.0, 0.0},      // set dumper ON
            {0, 0.0, 0.0, 0.0},       // go (0.0,0.0) and rotate 0
            {2, 0.0, 0.0, 90.0},      // rotate 90
            {2, 0.0, 0.0, 180.0},     // rotate 180
            {2, 0.0, 0.0, 270.0},     // rotate 270
            {2, 0.0, 0.0, 360},       // rotate 360
            {30,0.0,0.0, 0.0},        // Auto map builder
            {99,0.0,0.0, 0.0},        // end
};


GoalList turtlebot3_auto_map_achor[] ={
            {60, 0.0, 0.0, 0.0},      // course correct ON
            {64, 0.0, 0.0, 0.0},      // go curve ON
            {66, 0.0, 0.0, 0.0},      // force current position to map(0,0)
            {67, 0.0, 0.0, 0.0},      // set dumper ON
            {0, 0.0, 0.0, 0.0},       // go (0.0,0.0) and rotate 0
            {2, 0.0, 0.0, 90.0},      // rotate 90
            {2, 0.0, 0.0, 180.0},     // rotate 180
            {2, 0.0, 0.0, 270.0},     // rotate 270
            {2, 0.0, 0.0, 360},       // rotate 360
            {31,0.0,0.0, 0.0},        // Auto map builder of anchor
            {99,0.0,0.0, 0.0},        // end
};


GoalList turtlebot3_auto_map_localization[] ={
            {60, 0.0, 0.0, 0.0},      // course correct ON
            {64, 0.0, 0.0, 0.0},      // go curve ON
            {66, 0.0, 0.0, 0.0},      // force current position to map(0,0)
            {67, 0.0, 0.0, 0.0},      // set dumper ON
            {0, 0.0, 0.0, 0.0},       // go (0.0,0.0) and rotate 0
            {30,0.0,0.0, 0.0},        // Auto map builder
            {99,0.0,0.0, 0.0},        // end
};


int main(int argc, char** argv)
{
    //init the ROS node
    ros::init(argc, argv, "map_builder");
    ros::NodeHandle nh;
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

    MultiGoals mg_ex;
    mg_ex.init(nh);

    ros::Rate rate(1);   //  1[Hz]
    //for(int i=0;i<10;i++){
    //    ros::spinOnce();
    //    rate.sleep();
    //}

    //mg_ex.mloop_ex(turtlebot3_house);
    //mg_ex.mloop_ex(turtlebot3_auto_map);      // Auto Map
    mg_ex.mloop_ex(turtlebot3_auto_map_achor);  // Auto Map of Anchor

    //mg_ex.mloop_ex(turtlebot3_auto_map_localization);


    //mg_ex.mloop_ex(navi_list1);

    //unsigned char cost = mg_ex.drive.navi_.check_cost(-3.0,1.0);
    //std::cout <<"cost = "<< std::hex << (unsigned int)cost << std::endl;

    //ros::Rate rate(1);   //  1[Hz]
    rate.sleep();

    //std::exit(0);
    ros::spin();
    return 0;
    
}
