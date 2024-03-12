/*
* com_lib.hpp
*
*
*/

#ifndef COM_LIB_HPP
#define COM_LIB_HPP

//#include <ros/ros.h>
//#include <geometry_msgs/Twist.h>
//#include <tf/transform_listener.h>


//#include <unistd.h>


#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "tf2/exceptions.h"
#include "tf2/transform_datatypes.h"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "com_def.hpp"

//#include "rclcpp/clock.hpp"

/*
* T round_my(T dt,int n)
*/
template <class T=float>
T round_my(T dt,int n)
{
    if (n > 0){
        T x=(T)pow(10.0, n);
        dt *= x;
        return std::round(dt) / x;
    }
    else{
        return std::round(dt);
    }
}

class HeartBeat{
public:
    HeartBeat(){}
    void init(std::shared_ptr<rclcpp::Node> node);
    void set_on_off(bool act_on=true){
        act_on_=act_on;
    }

    bool act_on_;
    //volatile bool act_on_;

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr publisher_uint32_;
    u_int32_t no_;

    void timer_callback();

};

class GetTF
{

public:
    //! ROS node initialization
    GetTF(){}

    //void init(ros::NodeHandle &nh);
    void init(std::shared_ptr<rclcpp::Node> node);

    /*
    * void get(int func)
    */
    bool get(int func=0,const std::string& target_frame="map", const std::string& source_frame="base_footprint");
    void get2(int func=0,const std::string& target_frame="map", const std::string& source_frame="base_footprint");

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

    //tf::StampedTransform base_tf;
    tf2::Stamped<tf2::Transform> base_tf;
    double _rx, _ry, _rz;

private:
    //! The node handle we'll be using
    //ros::NodeHandle nh_;
    //! We will be listening to TF transforms as well
    //tf::TransformListener listener_;
    //geometry_msgs::Twist _vel_msg;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;        // http://docs.ros.org/en/jade/api/tf2_ros/html/c++/classtf2__ros_1_1Buffer.html

    //u_char log_level=3;
    u_char log_level=1;

    std::shared_ptr<rclcpp::Node> node_;
    rmw_qos_profile_t custom_qos_;

    bool use_sim_time_ = false;
    bool init_f = false;    // add by nishi 2024.2.27

};


#endif  // COM_LIB