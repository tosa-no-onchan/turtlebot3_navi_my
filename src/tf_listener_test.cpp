/*
* turtlebot3_navi_my/src/tf_listener_test.cpp
*
*/

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


#include "geometry_msgs/msg/twist.hpp"
//#include "geometry_msgs/msg/twist.hpp"

#include <rclcpp/publisher_options.hpp>

#include "turtlebot3_navi_my/com_lib.hpp"


int main(int argc, char * argv[])
{

  using namespace std::chrono_literals;

  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("tf_listener_test");
  //printf("%s",node);

  std::cout << "main() :#1" << std::endl;

  GetTF get_tf;

  std::cout << "main() :#2" << std::endl;

  get_tf.init(node);

  std::cout << "main() :#3" << std::endl;

  rclcpp::WallRate loop(1);
  while(rclcpp::ok()){
    //rclcpp::spin(std::make_shared<RobotDrive>());
    get_tf.get(2);
    std::cout << "loop" << std::endl;
  }
  rclcpp::shutdown();
  return 0;
}
