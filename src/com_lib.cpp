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

/*
* float normalize_tf_rz(float rz)
*  tf rz [radian] の補正
*   float rz: [Radian]
*   180度[Radian] 以上を補正します。
*  piを超す(WrapAround) と、符号が変わります。
*  TF rz は、常に 0pi から pi の間の角度と回転方向を返す。(0pi から近い角度)
*/
float normalize_tf_rz(float rz){
    //if(abs(rz) >= 360.0/RADIANS_F){
    //    if(rz >0.0)
    //        rz -= 360.0/RADIANS_F;
    //    else
    //        rz += 360.0/RADIANS_F;
    //}
    //if(abs(rz) >= 180.0/RADIANS_F){
    //    if(rz >0.0)
    //        rz -= 180.0/RADIANS_F;
    //    else
    //        rz += 180.0/RADIANS_F;
    //}

    rz= fmod(rz,(360.0/RADIANS_F));

    // 小数点以下5 の 丸めをしないと、うまく行かない。
    //rz = round_my<float>(rz,5);
    rz = round_my_zero<float>(rz);
    if(rz > 180.0/RADIANS_F){
        rz = -360.0/RADIANS_F + rz;
        //std::cout << "normalize_tf_rz() #2 rz=" << rz << " dz="<< rz * RADIANS_F << std::endl;
    }
    else if(rz < -180.0/RADIANS_F){
        rz = 360.0/RADIANS_F + rz;
        //std::cout << "normalize_tf_rz() #3 rz=" << rz << " dz="<< rz * RADIANS_F << std::endl;
    }
    return rz;
}

/*
* float normalize_tf_rz_quo(float rz, int &quo)
*  tf rz [radian] の補正
*   float rz: [Radian]
*   180度[Radian] 以上を補正します。
*   int &quo:   quotient (商)
*  piを超す(WrapAround) と、符号が変わります。
*  TF rz は、常に 0pi から pi の間の角度と回転方向を返す。(0pi から近い角度)
*/
float normalize_tf_rz_quo(float rz,int &quo){
    float quo_ = rz / (360.0/RADIANS_F);
    quo = (int)quo_;
    
    rz= fmod(rz,(360.0/RADIANS_F));

    // 小数点以下5 の 丸めをしないと、うまく行かない。
    //rz = round_my<float>(rz,5);
    rz = round_my_zero<float>(rz);
    if(rz == 0.0){
        if(quo > 0){
            quo--;
        }
        else if(quo <0)
            quo++;
        return rz;
    }
    return normalize_tf_rz(rz);
}

/*
* float reverse_tf_rz(float rz)
*   TF rz[radian] の反転
*   float rz: [Radian] の向きを反転させます。
*         rz(+)  -> rz(-)
*         rz(-)  -> rz(+)
*/
float reverse_tf_rz(float rz){
    rz = round_my_zero<float>(rz);
    if(rz > 0){
        rz = -360.0/RADIANS_F + rz;
    }
    else if(rz < 0){
        rz = 360.0/RADIANS_F + rz;
    }
    return rz;
}


/*
* float adjust_tf_rz(float stop_rz,float r_theta,float _rz)
*   TF rz[radian] の反転
*   float rz: [Radian] の向きを反転させます。
*         rz(+)  -> rz(-)
*         rz(-)  -> rz(+)
*/
float adjust_tf_rz(float stop_rz,float r_theta,float _rz){
    // 目的角までの差角を求める
    //float r_theta = stop_rz - _rz;
    // stop_rz、_rz の回転角 空間が異なる場合、補正が必要
    // stop_rz は、left(+)回転 空間
    if(stop_rz >= 0){
        // _rz は、right(-)回転 空間
        if(_rz < 0){
            std::cout << " #4 passed !!" << std::endl;
            //r_theta = stop_rz + _rz;
            r_theta = stop_rz - (360/RADIANS_F + _rz);
        }
    }
    // stop_rz は、right(-)回転 空間
    else{
        // _rz は、left(+)回転 空間
        if(_rz >= 0){
            // _rz(+) を、 _rz(-) に回転座標変換して、 stop_rz(-) を追い越さない場合、言い換えると
            //     |_rz(-)|  <=  |stop_rz(-)| 
            // の時は、下記計算が使える .... 計算しても符号(-)が変わらない事を条件とする
            //r_theta = stop_rz - (-360/RADIANS_F + _rz);
            float rz_minus = -360/RADIANS_F + _rz;
            rz_minus = round_my_zero<float>(rz_minus);
            if(abs(rz_minus) <= abs(stop_rz)){
                std::cout << " #5 passed !!" << std::endl;
                // _rz(-) が stop_rz(-) をおい越さ無い場合
                r_theta = stop_rz - rz_minus;
            }
            else{
                std::cout << " #6 passed !!" << std::endl;
                // _rz(-) が、stop_rz(-) をおい越す場合
                r_theta = stop_rz + _rz;
            }
        }
    }
    return r_theta;
}

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
        // https://answers.ros.org/question/339528/quaternion-to-rpy-ros2/
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

/**
* @brief フォルダ以下のファイル一覧を取得する関数
* @param[in]    folderPath  フォルダパス
* @param[out]   file_names  ファイル名一覧
* return        true:成功, false:失敗
*
* https://phst.hateblo.jp/entry/2019/02/17/003027
* https://qiita.com/tes2840/items/8d295b1caaf10eaf33ad
* sort
* https://cpprefjp.github.io/reference/algorithm/sort.html
*/
bool getFileNames(std::string folderPath, std::vector<std::string> &file_names)
{
    //using namespace std::filesystem;
    namespace fs = std::filesystem;

    fs::path path1(folderPath);

    // https://en.cppreference.com/w/cpp/filesystem/path/filename

    //ファイルをリスト化
    if (fs::is_directory(path1)) {
        //std::cout << "Directory: " << fs::absolute(path1).string() << std::endl;
        //std::cout << "Files: " << std::endl;
        auto dir_it = fs::directory_iterator(path1);
        for (auto &p : dir_it) {
            std::string s = p.path().filename().string();
            //std::cout <<" p.path().filename().string():"<< s << std::endl;
            file_names.push_back(s);
        }
    }
    else
        return false;

    return true;
}