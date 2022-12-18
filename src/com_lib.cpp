/*
* com_lib.cpp
*
* https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Cpp.html
* https://qiita.com/NeK/items/bcf518f6dd79f970bb8e
*/

#include "turtlebot3_navi_my/com_lib.hpp"


//#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::chrono_literals;
//using geometry_msgs::msg::Quaternion;


//void GetTF::init(ros::NodeHandle &nh){
void GetTF::init(std::shared_ptr<rclcpp::Node> node){
    //nh_=nh;
    node_=node;

    rclcpp::Rate rate(1);

    //wait for the listener to get the first message
    //listener_.waitForTransform("base_footprint","odom", ros::Time(0), ros::Duration(1.0));
    //listener_.waitForTransform("base_footprint","map", ros::Time(0), ros::Duration(1.0));

    //listener_.waitForTransform("map","base_footprint", ros::Time(0), ros::Duration(1.0));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    rate.sleep();
}
/*
* https://docs.ros2.org/latest/api/geometry_msgs/msg/TransformStamped.html
* https://docs.ros2.org/latest/api/geometry_msgs/msg/Transform.html
*
* http://wiki.ros.org/tf2/Tutorials/Migration/DataConversions
* https://answers.ros.org/question/280781/transform-geometry_msgstransformstamped-to-tf2transform-transform/
*
* GetTF::get(int func)
*  func: 0/1/2 or 10/11/12
*     0/1/2 -> tf chain "map", "base_footprint"
*     10/11/12 -> tf chain "base_footprint","map"  not use
*/
void GetTF::get(int func){
    //ros::Rate rate(70.0);
    rclcpp::WallRate rate(70.0);
    int cnt=5;
    bool rc=true;
    geometry_msgs::msg::TransformStamped tf;
    while (1){
        try{
            //listener_.lookupTransform("base_footprint","odom", ros::Time(0), base_tf);
            //listener_.lookupTransform("base_footprint","map", ros::Time(0), base_tf);
            //listener_.lookupTransform("map","base_footprint", ros::Time(0), base_tf);

            /* lookupTransform(const std::string& target_frame, const std::string& source_frame,
		     *    const TimePoint& time);
             *
             * \brief Get the transform between two frames by frame ID.
             * \param target_frame The frame to which data should be transformed
             * \param source_frame The frame where the data originated
             * \param time The time at which the value of the transform is desired. (0 will get the latest)
             * \return The transform between the frames
             *
             * Possible exceptions tf2::LookupException, tf2::ConnectivityException,
             * tf2::ExtrapolationException, tf2::InvalidArgumentException
             *
             * lookupTransform(const std::string& target_frame, const std::string& source_frame,
		     *    const TimePoint& time);
            */
            //if (func < 10){
                tf = tf_buffer_->lookupTransform("map", "base_footprint",tf2::TimePointZero);
            //}
            //else{
            //    tf = tf_buffer_->lookupTransform("base_footprint","map",tf2::TimePointZero);
            //}
            tf2::fromMsg(tf, base_tf);
        }
        //catch (tf::TransformException ex){
        catch (const tf2::TransformException & ex){
            //ROS_ERROR("%s",ex.what());
            cnt--;
            if(cnt<=0){
                rc=false;
                break;
            }
            rate.sleep();
            continue;
        }
        break;
    }
    if (rc==true && (func==2 || func==12) ){
        //Quaternion getRotation()
        //tf::Quaternion q = base_tf.getRotation();
        tf2::Quaternion q = base_tf.getRotation();

        //tf::Matrix3x3 m(q);
        tf2::Matrix3x3 m(q);
        //double roll, pitch, yaw;
        m.getRPY(_rx, _ry, _rz);

        if(log_level>=3){
            double qw_ = tf.transform.rotation.w;
            double qx_ = tf.transform.rotation.x;
            double qy_ = tf.transform.rotation.y;
            double qz_ = tf.transform.rotation.z;
            std::cout << "qw_: " << qw_ << ", qx_: " << qx_ << ", qy_: " << qy_ << ", qz_:"<< qz_ <<std::endl;
        }
        if(log_level>=3)
            std::cout << "_rx: " << _rx << ", _ry: " << _ry << ", _rz: " << _rz << std::endl;
    }
}