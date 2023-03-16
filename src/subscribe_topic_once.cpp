/*
*  [ROS2] C++で一度だけトピックを購読する
*  https://qiita.com/buran5884/items/9ee9b1608716233a9873
*/

//#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
//#include <sensor_msgs/msg/image.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <chrono>

int main(int argc, char* argv[]){
  using namespace std::chrono_literals;
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("subscribe_topic_once");

  //sensor_msgs::msg::Image image_msg;
  nav_msgs::msg::OccupancyGrid map;

  std::string topic_name = "map";
  auto timeout = std::chrono::milliseconds(100);

  //bool is_successful = rclcpp::wait_for_message(image_msg, node, topic_name, timeout);
  bool is_successful = rclcpp::wait_for_message<nav_msgs::msg::OccupancyGrid>(map, node, topic_name, timeout);
  if(is_successful){
    // 購読に成功したときの処理
    RCLCPP_INFO(node->get_logger(), "SUCCESS");
  }else{
    // 購読に失敗したときの処理
    RCLCPP_ERROR(node->get_logger(), "FAILURE");
  }

 return 0;
}

