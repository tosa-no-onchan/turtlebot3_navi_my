/*
drive_base.cpp
https://wiki.ros.org/pr2_controllers/Tutorials/Using%20the%20base%20controller%20with%20odometry%20and%20transform%20information

https://www.k-cube.co.jp/wakaba/server/func/math_h.html

https://answers.ros.org/question/50113/transform-quaternion/

build
$ catkin_make --pkg turtlebot3_navi_my

$ rosrun turtlebot3_navi_my drive_base
*/

#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>

#include <unistd.h>

//#include <math.h>

#include "turtlebot3_navi_my/robot_drive.h"

int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "robot_driver");
  ros::NodeHandle nh;

  RobotDrive drive;

  drive.init(nh);
  //driver.driveForwardOdom(0.5);
  //driver.turnOdom(true, 0.01);

  int test_id=1;

  //Testing our function
  if (test_id==1){
    drive._course_correct=true;
    drive.go_abs(1.0,0.0);
  
  }

  if (test_id==2){
    drive.rotate_abs(-30.0,5.0);
    //drive.rotate_abs(-25.0,5.0);

    //drive.rotate_abs(30.0,5.0);
    //drive.rotate_abs(25.0,5.0);

    //drive.rotate_abs(180.0,5.0);
    //drive.rotate_abs(175.0,5.0);


    //drive.rotate_abs(-180.0,5.0);
    //drive.rotate_abs(-175.0,5.0);


    //drive.rotate_abs(179.0,5.0);
    //drive.rotate_abs(190.0,5.0);

    //drive.rotate_abs(180.0,5.0);
    //drive.rotate_abs(-180.0,5.0);
    //drive.rotate_abs(270.0,5.0);

    //driver.get_tf(2);
    std::cout << "dx,dy,dz=" << drive.round_my<double>(drive._rx*RADIANS_F,3) << "," << drive.round_my<double>(drive._ry*RADIANS_F,3)  
      << "," << drive.round_my<double>(drive._rz*RADIANS_F,3) << std::endl;

  }

  if (test_id==3){
      drive.rotate_off(-5.0,3.0);
      drive.get_tf(2);
      std::cout << "dx,dy,dz=" << drive.round_my<double>(drive._rx*RADIANS_F,3) << "," << drive.round_my<double>(drive._ry*RADIANS_F,3)  
        << "," << drive.round_my<double>(drive._rz*RADIANS_F,3) << std::endl;

      //drive.rotate_off(-5.0,3.0);
      //drive.get_tf(2);

      //std::cout << "dx,dy,dz=" << drive.round_my<double>(drive._rx*RADIANS_F,3) << "," << drive.round_my<double>(drive._ry*RADIANS_F,3)  << "," << drive.round_my<double>(drive._rz*RADIANS_F,3) << std::endl;

  }

  ros::Rate rate(1);   //  1[Hz]
  rate.sleep();

  std::exit(0);
}
