/*
* ROS2 robot_driveCmd_Vel.hpp
* https://wiki.ros.org/pr2_controllers/Tutorials/Using%20the%20base%20controller%20with%20odometry%20and%20transform%20information
*
* https://www.k-cube.co.jp/wakaba/server/func/math_h.html
*
* https://answers.ros.org/question/50113/transform-quaternion/
*
* build
* $ catkin_make --pkg turtlebot3_navi_my
*
* $ rosrun turtlebot3_navi_my drive_base
*/

#ifndef ROBOT_DRIVE_CMD_VEL_HPP
#define ROBOT_DRIVE_CMD_VEL_HPP

//#include <iostream>

//#include <ros/ros.h>
//#include <geometry_msgs/Twist.h>
//#include <tf/transform_listener.h>

//#include "turtlebot3_navi_my/robot_navi.hpp"
#include "turtlebot3_navi_my/com_lib.hpp"
#include "turtlebot3_navi_my/robot_driveCore.hpp"

//#include <unistd.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"


#include <rclcpp/publisher_options.hpp>


//#include <math.h>

// https://www.k-cube.co.jp/wakaba/server/func/math_h.html
//#define deg_to_rad(deg) (((deg)/360)*2*M_PI)
//#define rad_to_deg(rad) (((rad)/2/M_PI)*360)

#ifndef RADIANS_F
#define RADIANS_F   57.29577951308232    // [deg/rad]
#endif

class RobotDriveCmd_Vel: public Robot_DriveCore
{

public:
  //tf2::Stamped<tf2::Transform> base_tf;

  //RobotNavi navi_;
  //GetTF getTF_;
  GetTF *getTF_;   // changed by nishi 2024.2.27

  HeartBeat heartBeat_;   // add by nishi 2023.3.8

  //double _rx, _ry, _rz;
  //bool _course_correct;
  //bool _after_correct_wait;
  //bool _go_curve;
  //bool _dumper;

  //! ROS node initialization
  RobotDriveCmd_Vel(){}

  void init(std::shared_ptr<rclcpp::Node> node,GetTF *getTF,bool navi_use=false);

  /*
  move()
  自分からの相対位置へ移動
      float dist: 自分からの距離
                    > 0 前進
                    < 0 後退
      float d_yaw: ロボットからの角度。 [degree]  > 0 左回転 /  < 0 右回転
  */
  void move(float dist,float d_yaw);

  /*
  move_abs()
      x,y: 絶対番地への移動(基準座標)
      d_yaw: 基準座標での角度。 [degree]  > 0 左回転 /  < 0 右回転
  */
  void move_abs(float x,float y,float d_yaw);

  /*
  comp_dad() : compute distanse and direction
  目的地までの距離と方角を計算する。
      float x:
      float y:
      float &dist:
      float &r_yaw:
      float &r_yaw_off:
  */
  void comp_dad(float x,float y,float &dist, float &r_yaw, float &r_yaw_off);

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
  * bool get_tf(int func)
  */
  bool get_tf(int func=0);
  /*
  go_abs(x,y,isBack=false,speed=0.05)
  直進する。
  */
  void go_abs(float x,float y,bool isback=false,float speed=0.05 );

  /*
  rotate_abs()
      stop_dz(d_theta) : [deg] 基本座標上の角度  > 0 左回転 /  < 0 右回転
      rad_f : false -> deg / true -> radian
      speed :  15.0  [deg/s]
  */
  //void rotate_abs(float stop_dz,float speed=5.0);
  //void rotate_abs(float stop_dz,float speed=8.0);
  void rotate_abs(float stop_dz,bool rad_f=false, float speed=15.0);

  /*
  rotate_abs_179()
    最後の 179[deg] を回転する
      stop_dz(d_theta) : [deg] 基本座標上の角度  > 0 左回転 /  < 0 右回転
      rad_f : false -> deg / true -> radian
      speed :  15.0  [deg/s]
  */
  void rotate_abs_179(float stop_dz,bool rad_f=false, float speed=15.0);

  /* 
  void rotate_off()
      d_theta : [deg] ロボットの今の向きからの角度   > 0 左回転 /  < 0 右回転
      speed :  15.0  [deg/s]
  */
  void rotate_off(float d_theta, float speed=15.0,bool go_curve=false);

  //! Drive forward a specified distance based on odometry information
  bool driveForwardOdom(double distance);
  bool turnOdom(bool clockwise, double radians);


  bool navi_move(float x,float y,float r_yaw,float r_yaw_off=0.0);
  void navi_map_save();

private:
  //! The node handle we'll be using
  //ros::NodeHandle nh_;
  std::shared_ptr<rclcpp::Node> node_;

  //! We will be publishing to the "cmd_vel" topic to issue commands
  //ros::Publisher _pub;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> _pub;

  geometry_msgs::msg::Twist _vel_msg;

  u_char log_level=1;


  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;


};

#endif      // ROBOT_DRIVE_HPP
