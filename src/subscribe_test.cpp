
/*
* https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html
*
* $ ros2 run turtlebot3_navi_my subscribe_test
* $ ros2 launch turtlebot3_navi_my subscribe_test.launch.py use_sim_time:=True
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
      //#define ORG_TEST
      #if defined(ORG_TEST)
        subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
          msg_id, 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
      #else
        subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
          msg_id, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(), std::bind(&MinimalSubscriber::topic_callback, this, _1));

      #endif
    }
    int cnt=0;
  private:
    //std::string msg_id="map";
    std::string msg_id="local_costmap/costmap";
    void topic_callback(const nav_msgs::msg::OccupancyGrid & map_msg)
    {
      //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
      cnt++;
      std::cout << cnt << ":map_msg.header.frame_id:" << map_msg.header.frame_id << std::endl;
      // msg_id="local_costmap/costmap"
      // [subscribe_test-1] 28:map_msg.header.frame_id:map
    }
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  std::cout << "Start subscribe_test" << std::endl;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
