
/*
* https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html
*/
#include <memory>

#include "rclcpp/rclcpp.hpp"
//#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber() : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }
    int cnt=0;
  private:
    void topic_callback(const nav_msgs::msg::OccupancyGrid & map_msg)
    {
      //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
      cnt++;
      std::cout << cnt << ":map_msg.header.frame_id" << map_msg.header.frame_id << std::endl; 
    }
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
