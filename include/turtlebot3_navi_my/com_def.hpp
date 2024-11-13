/*
* com_def.hpp
*/

#ifndef COM_DEF_HPP
#define COM_DEF_HPP


#include <cstdio>
//#include <ros/ros.h>
#include "rclcpp/rclcpp.hpp"


#ifndef RADIANS_F
#define RADIANS_F   57.29577951308232    // [deg/rad]
#endif

/*-----------------------
Point
------------------------*/
typedef struct{
    float x;
    float y;
} Point;

/*-----------------------
BorderBox
------------------------*/
typedef struct{
    Point top_r;    // top-right
    Point bot_l;    // bottom-left
} BorderBox;
/*-----------------------
MapM
http://docs.ros.org/en/indigo/api/nav_msgs/html/msg/MapMetaData.html
------------------------*/
typedef struct{
    float resolution;
    int width;      // Map image width   add by nishi 2022.8.18
    int height;     // Map image height  add by nishi 2022.8.18
    float origin[3];    // [x:0.000000, y:0.000000, yaw:0.000000]
} MapM;

/*-----------------------
Yaml
------------------------*/
//typedef struct{
//    float resolution;
//    float origin[3];    // [x:0.000000, y:0.000000, yaw:0.000000]
//    int img_width;      // Map image width   add by nishi 2022.8.18
//    int img_height;     // Map image height  add by nishi 2022.8.18
//} Yaml;

/*------------------------------------------------
 GoalList
  func,x,y,d_yaw
  func: 0 -> move point x,y and rotate d_yaw
       1 -> move point x,y only
       2 -> rotate d_yaw only
       21 -> sleep
       22 -> get map
       50 -> set Navigation mode
       99 -> end
--------------------------------------------------*/
typedef struct {
    u_int8_t func;
    float x;
    float y;
    float d_yaw;
} GoalList;

/*------------------------------------------------
 GoalList2
  func,x,dist,d_yaw
  func: 0 -> move forward dist and rotate d_yaw  x > 0: go forward, x < 0 : go back(cmd_vel only)
       21 -> sleep
       22 -> get map
       50 -> set Navigation mode
       99 -> end
--------------------------------------------------*/
typedef struct {
    u_int8_t func;
    float dist;
    float d_yaw;
} GoalList2;


typedef struct {
    int xi;
    int yi;
    float x;
    float y;
    float dist;
    int pic;
} Gpoint;


#endif      // COM_DEF_HPP
