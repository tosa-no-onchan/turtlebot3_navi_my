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

void HeartBeat::init(std::shared_ptr<rclcpp::Node> node){
    //nh_=nh;
    node_=node;
    act_on_=true;

    //std::chrono::milliseconds ms250(250);
    std::chrono::milliseconds ms250(10000);

    //publisher_uint32_ = node_->create_publisher<std_msgs::msg::UInt32>("fox_beat", 10);
    publisher_uint32_ = node_->create_publisher<std_msgs::msg::UInt32>("pc_beat", 10);  // changed by nishi 2023.3.15
    timer_ = node_->create_wall_timer(
        //250ms, std::bind(&HeartBeat::timer_callback, this));
        ms250, std::bind(&HeartBeat::timer_callback, this));
}

//bool HeartBeat::set_on_off(bool act_on){
//    if(act_on==true)
//        std::cout << "heartBeat_.set_on_off(true)"<< std::endl;
//    else
//        std::cout << "heartBeat_.set_on_off(false)"<< std::endl;
//    act_on_=act_on;
//    std::cout << "heartBeat_.set_on_off():99 end!"<< std::endl;
//}


void HeartBeat::timer_callback(){
    no_++;
    bool act_on;
    act_on = act_on_;
    if(act_on == true){
        auto m_uint32 = std_msgs::msg::UInt32();
        m_uint32.data = no_;
        publisher_uint32_->publish(m_uint32);
    }
}


//void GetTF::init(ros::NodeHandle &nh){
void GetTF::init(std::shared_ptr<rclcpp::Node> node){
    if(init_f == true){
        return;
    }
    init_f=true;
    //nh_=nh;
    node_=node;

    //node_->get_parameter<bool>("use_sim_time",use_sim_time_);

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
* https://docs.ros2.org/foxy/api/tf2/classtf2_1_1BufferCore.html
*
* bool GetTF::get(int func)
*  func: 0/1/2 or 10/11/12
*     0/1/2 -> tf chain "map", "base_footprint"
*/
bool GetTF::get(int func,const std::string& target_frame, const std::string& source_frame){
    //ros::Rate rate(70.0);
    rclcpp::WallRate rate(70.0);
    int cnt=70;
    bool rc=true;
    geometry_msgs::msg::TransformStamped tf;
    tf2::Duration dt(0); // nanosec 600[ms] 600*1,000,000 [nsec]

    //rclcpp::Duration timeout = rclcpp::Duration::from_nanoseconds(1000000);
    rclcpp::Duration timeout = rclcpp::Duration::from_nanoseconds(0);

    // https://cpprefjp.github.io/reference/chrono/time_point/op_constructor.html
    auto st = tf2::get_now();
    tf2::TimePoint tp(st.time_since_epoch()-dt);

    // https://wazalabo.com/ros2_timer.html
    // Node::now()で使用されるクロックは rclcpp/node_clock.cpp で初期化している通り ROS Time を使用しており、 /clock トピックへの publish がない場合は System Time として振る舞います。
    rclcpp::Time tm = node_->now();

    // rclcpp::Time
    //rclcpp::Time tm2 = node_->get_clock()->now();

    //printf("%s",tm2);

    while (1){
        try{
            //rclcpp::spin_some(node_);
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
            //tf = tf_buffer_->lookupTransform("map", "base_footprint",tf2::TimePointZero);
         
            //tf = tf_buffer_->lookupTransform(target_frame, source_frame,tf2::TimePointZero);
            
            // galactic here
            //tf = tf_buffer_->lookupTransform(target_frame, source_frame,tp);

            // humble here
            // lookupTransform(
            //      const std::string & target_frame, const std::string & source_frame,
            //      const rclcpp::Time & time,
            //      const rclcpp::Duration timeout = rclcpp::Duration::from_nanoseconds(0)) const
            //tf = tf_buffer_->lookupTransform(target_frame, source_frame,tp,dt);
            tf = tf_buffer_->lookupTransform(target_frame, source_frame,tm,timeout);


            tf2::fromMsg(tf, base_tf);
        }
        //catch (tf::TransformException ex){
        catch (const tf2::TransformException & ex){
            //ROS_ERROR("%s",ex.what());
            //RCLCPP_ERROR(node_->get_logger(), "%s",ex.what());
            //std::cout << "GetTF::get():#2 error " << std::endl;
            cnt--;
            if(cnt<=0){
                std::cout << "GetTF::get():#3 error " << std::endl;
                //RCLCPP_ERROR(node_->get_logger(), "%s",ex.what());
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
    return rc;
}

void GetTF::get2(int func,const std::string& target_frame, const std::string& source_frame){
    //ros::Rate rate(70.0);
    rclcpp::WallRate rate(70.0);
    int cnt=5;
    bool rc=true;
    geometry_msgs::msg::TransformStamped tf;

    rclcpp::Duration timeout = rclcpp::Duration::from_nanoseconds(1000000);

    // https://wazalabo.com/ros2_timer.html
    // Node::now()で使用されるクロックは rclcpp/node_clock.cpp で初期化している通り ROS Time を使用しており、 /clock トピックへの publish がない場合は System Time として振る舞います。
    rclcpp::Time tm = node_->now();

    //tf2::TimePoint tn;

    while (1){
        try{
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
            //tf = tf_buffer_->lookupTransform("map", "base_footprint",tf2::TimePointZero);
            //tf = tf_buffer_->lookupTransform(target_frame, source_frame,tf2::TimePointZero);
            //tf2::TimePoint tn;
            // here galactic
            //tf = tf_buffer_->lookupTransform(target_frame, tn, source_frame, tn, "map");

            // here humble
            // lookupTransform(const std::string&, const tf2::TimePoint&,
            //                 const std::string&, const tf2::TimePoint&,
            //                 const std::string&, const tf2::Duration)
            //tf2::Duration dt(0); // nanosec 600[ms] 600*1,000,000 [nsec]
            tf = tf_buffer_->lookupTransform(target_frame, source_frame,tm,timeout);


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

