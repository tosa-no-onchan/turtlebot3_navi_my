/*
robot_drive.h
https://wiki.ros.org/pr2_controllers/Tutorials/Using%20the%20base%20controller%20with%20odometry%20and%20transform%20information

https://www.k-cube.co.jp/wakaba/server/func/math_h.html

https://answers.ros.org/question/50113/transform-quaternion/

build
$ catkin_make --pkg turtlebot3_navi_my

$ rosrun turtlebot3_navi_my drive_base
*/

#ifndef ROBOT_DRIVE_H
#define ROBOT_DRIVE_H

#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>

#include "turtlebot3_navi_my/robot_navi.h"

#include <unistd.h>

//#include <math.h>

// https://www.k-cube.co.jp/wakaba/server/func/math_h.html
//#define deg_to_rad(deg) (((deg)/360)*2*M_PI)
//#define rad_to_deg(rad) (((rad)/2/M_PI)*360)

#define RADIANS_F   57.29577951308232    // [deg/rad]

class RobotDrive
{
private:
  //! The node handle we'll be using
  ros::NodeHandle nh_;
  //! We will be publishing to the "cmd_vel" topic to issue commands
  ros::Publisher _pub;
  //! We will be listening to TF transforms as well
  tf::TransformListener listener_;
  tf::StampedTransform base_tf;
  geometry_msgs::Twist _vel_msg;

public:

  double _rx, _ry, _rz;
  bool _course_correct;
  bool _after_correct_wait;

  //! ROS node initialization
  RobotDrive(){}
  void init(ros::NodeHandle &nh);

  /*
  move()
  自分からの相対位置へ移動
      dist: 自分からの距離
      d_yaw: 基準座標での角度。 [degree] ロボット座標上の角度では無い
  */
  void move(float dist,float d_yaw);
  /*
  move_abs()
      x,y: 絶対番地への移動(基準座標)
      d_yaw: 基準座標での角度。 [degree]
  */
  void move_abs(float x,float y,float d_yaw);

  /*
  * T round_my(T dt,int n)
  */
  template <class T=float>
  T round_my(T dt,int n){
    if (n > 0){
      T x=(T)pow(10.0, n);
      dt *= x;
      return std::round(dt) / x;
    }
    else{
      return std::round(dt);
    }
  }
  /*
  * void get_tf(int func)
  */
  void get_tf(int func=0);
  /*
  go_abs(x,y,isForward=True,speed=0.05)
  直進する。
  */
  void go_abs(float x,float y,float speed=0.05 ,bool isForward=true);
  /*
  rotate_abs()
      stop_dz(d_theta) : [deg] 基本座標上の角度
      speed :  5.0  [deg/s]
  */
  void rotate_abs(float stop_dz,float speed=5.0);
  /* 
  void rotate_off()
      d_theta : [deg] ロボット座標上の角度
      speed :  5.0  [deg/s]
  */
  void rotate_off(float d_theta, float speed=5.0);
  //! Drive forward a specified distance based on odometry information
  bool driveForwardOdom(double distance);
  bool turnOdom(bool clockwise, double radians);
};

#endif      // ROBOT_DRIVE_H
