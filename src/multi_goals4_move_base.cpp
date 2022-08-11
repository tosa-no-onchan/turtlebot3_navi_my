/*
multi_goals4_move_base.cpp


https://robot.isc.chubu.ac.jp/?p=1261
https://qiita.com/nnn112358/items/d159204d565469f647bb
https://inomacreate.com/original-robot04/


build
$ catkin build turtlebot3_navi_my

$ rosrun turtlebot3_navi_my multi_goals4_move_base
*/

#include <iostream>

#include <ros/ros.h>
#include <ros/topic.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include "std_srvs/Empty.h"
#include <unistd.h>

//#include <math.h>


//#define USE_MOVE_BASE
#include "turtlebot3_navi_my/multi_goals.h"


#include <nav_msgs/OccupancyGrid.h>




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


int main(int argc, char **argv){
    //ros::init(argc, argv, "set_goal");
    ros::init(argc, argv, "muliti_goals4_move_base");

    ros::NodeHandle nh;

    MultiGoals mg_ex;
    mg_ex.init(nh);

    ros::Rate rate(1);   //  1[Hz]

    //mg_ex.mloop_ex(turtlebot3_house);
    mg_ex.mloop_ex(turtlebot3_auto_map);

    ros::spinOnce();

    ros::spin();

    return 0;
}
