/*
* ROS2
* drive_base.cpp
*
* https://github.com/ros2/turtlebot2_demo/blob/master/turtlebot2_follower/src/follower.cpp
* https://github.com/ros2/turtlebot2_demo/blob/master/turtlebot2_drivers/src/dumb_teleop.cpp
* https://zenn.dev/uchidaryo/articles/ros2-programming-6
*/
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


#include "geometry_msgs/msg/twist.hpp"
//#include "geometry_msgs/msg/twist.hpp"

#include <rclcpp/publisher_options.hpp>

#include "turtlebot3_navi_my/robot_driveCmd_Vel.hpp"
//#include "turtlebot3_navi_my/robot_driveNAV2.hpp"


int main(int argc, char * argv[])
{

  using namespace std::chrono_literals;

  rclcpp::init(argc, argv);
  //rclcpp::spin(std::make_shared<RobotDrive>());

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("drive_base");

  //std::shared_ptr<GetTF>getTF; 
  //getTF=std::shared_ptr<GetTF>(new GetTF);
  GetTF getTF; 

  RobotDriveCmd_Vel drive;
  //RobotDriveNAV2 drive;

  drive.init(node,&getTF,false);

  int test_id=1;

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

  rclcpp::WallRate loop(1);
  while(rclcpp::ok()){
    //rclcpp::spin(std::make_shared<RobotDrive>());
    //get_tf.get(2);
    std::cout << "loop" << std::endl;
    loop.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
