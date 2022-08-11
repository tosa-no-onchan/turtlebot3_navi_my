/*
* com_def.h
*/

#ifndef COM_DEF_H
#define COM_DEF_H


#include <cstdio>
#include <ros/ros.h>

#ifndef RADIANS_F
#define RADIANS_F   57.29577951308232    // [deg/rad]
#endif


/*-----------------------
Yaml
------------------------*/
typedef struct{
    float resolution;
    float origin[3];    // [0.000000, 0.000000, 0.000000]    
} Yaml;

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
  func: 0 -> move dist and rotate d_yaw
       1 -> move point x,y only
       2 -> rotate d_yaw only
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
    float x;
    float y;
    float dist;
    int pic;
} Gpoint;


#endif      // COM_DEF_H
