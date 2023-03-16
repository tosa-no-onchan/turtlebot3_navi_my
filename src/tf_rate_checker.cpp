/*
* turtlebot3_navi_my/src/tf_rate_checker.cpp
*
* run
* ros2 run turtlebot3_navi_my tf_rate_checker
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
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("tf_rate_checker");
  //printf("%s",node);

  std::cout << "main() :#1" << std::endl;

  GetTF get_tf;

  std::cout << "main() :#2" << std::endl;

  get_tf.init(node);

  std::cout << "main() :#3" << std::endl;

  rclcpp::WallRate loop(1);

  int cnt=0;

  rclcpp::Time prev_time = node->now();
  double off_t=5.0;

  while(rclcpp::ok()){
    //rclcpp::spin(std::make_shared<RobotDrive>());
    get_tf.get2(1,"map","odom");
    cnt++;
    rclcpp::Duration dt = node->now() - prev_time;
    if(dt.seconds() >= off_t){
      int rate = cnt / (int)off_t;
      std::cout << "rate=" << rate << std::endl;
      cnt=0;
      prev_time = node->now();
    }
  }
  rclcpp::shutdown();
  return 0;
}
