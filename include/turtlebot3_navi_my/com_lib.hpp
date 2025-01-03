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
#include <math.h>

#include <filesystem>

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


/*
* T round_my_zero(T dt,int n)
*/
template <class T=float>
T round_my_zero(T dt)
{
    return round_my<T>(dt,5);
}


/*
* float normalize_tf_rz(float rz)
*  tf rz [radian] の補正
*   float rz: [Radian]
*   180度[Radian] 以上を補正します。  
*  piを超す(WrapAround) と、符号が変わります。
*  TF rz は、常に 0pi から pi の間の角度と回転方向を返す。(0pi から近い角度)
*/
float normalize_tf_rz(float rz);

/*
* float normalize_tf_rz_quo(float rz,int &quo)
*  tf rz [radian] の補正
*   float rz: [Radian]
*   180度[Radian] 以上を補正します。  
*   int &quo:   quotient (商)
*  piを超す(WrapAround) と、符号が変わります。
*  TF rz は、常に 0pi から pi の間の角度と回転方向を返す。(0pi から近い角度)
*/
float normalize_tf_rz_quo(float rz,int &quo);

/*
* float reverse_tf_rz(float rz)
*   TF rz[radian] の反転
*   float rz: [Radian] の向きを反転させます。
*         rz(+)  -> rz(-)
*         rz(-)  -> rz(+)
*/
float reverse_tf_rz(float rz);


float adjust_tf_rz(float stop_rz,float r_theta,float _rz);

/**
* @brief フォルダ以下のファイル一覧を取得する関数
* @param[in]    folderPath  フォルダパス
* @param[out]   file_names  ファイル名一覧
* return        true:成功, false:失敗
*
* https://phst.hateblo.jp/entry/2019/02/17/003027
* https://qiita.com/tes2840/items/8d295b1caaf10eaf33ad
*/
bool getFileNames(std::string folderPath, std::vector<std::string> &file_names);

struct Path_val {
    int x,y;        // off set x,y [dot]
    float x_r,y_r;  // off set tf real [M]
    float theta_r;  // off set [radian]
    bool valid;
    float x_ar,y_ar;  // aboslute tf real [M]
};

class Path_plan{
public:
    std::vector<Path_val> path_val_;
    std::vector<Path_val> path_val_conde_;

    // sx,sy -> start : roboto の、現在位置 (tf real 空間)
    float sx_,sy_;
    float off_x_,off_y_;

    float master_res_ = 0.05;
    float res_ = 0.025;

    float theta_r_;
    int cur_p=0;

    bool y_revert_;
    bool plan_done_ = false;

    Path_plan(){}

    void reset(){
        if(path_val_.empty() == false){
            path_val_.clear();
        }
        if(path_val_conde_.empty() == false){
            path_val_conde_.clear();
        }
        condense_done=false;
        plan_done_=false;
        cur_p=0;
    }

    /*
    * sx,sy -> start : roboto の、現在位置 (tf real 空間)
    *
    * y_revert : false -> cost map => cv::Mat 変換時に、 y軸を逆転させていない。見た目の上下が同じ。
    *           true -> y軸を逆転 させる。 通常の方法
    */
    void init(float sx, float sy, float off_x, float off_y, bool y_revert=false ,float master_res=0.05,float res=0.025){
        sx_=sx;
        sy_=sy;
        off_x_ = off_x;
        off_y_ = off_y;
        master_res_=master_res;
        res_= res;
        y_revert_ = y_revert;

        // 目的地の方角 on tf
        theta_r_ = std::atan2(off_y,off_x);   //  [radian]
        reset();
    }
    /*
    * void append(int x, int y, bool valid=true)
    *   int x: opp_with_lstm predict
    * 
    *  opp_tflite predict 結果の、 x,y を、tf-real に変換して path_val_ に登録する。
    * 
    *  注1) 今は、下記計算が加味されていません。 by nishi 2024.12.26
    *  Mat map 座標 -> real world 座標に変換 の処理は、下記を参考にする。
    *  gridToWorld(int gx, int gy, float& wx, float& wy,MapM& mapm)
    *    wx = ((float)gx + 0.5) * mapm.resolution + mapm.origin[0];
    *    wy = ((float)gy + 0.5) * mapm.resolution + mapm.origin[1];      // y軸が反転している場合。 
    *    wy = ((float)(yaml.img_height-gy) + 0.5) * yaml.resolution + yaml.origin[1];   <-- y軸が反転していない場合。
    * 
    *  注2) 今回は、 sx_, sy_ 自体が、 tf-real 空間の値だから、
    *    そこからのオフセットを求めるから、上記 
    *       + contbuilder_.mapm_.origin[0]
    *       + contbuilder_.mapm_.origin[1]
    *    は、必要無い。
    */
    void append(int x, int y, bool valid=true){
        Path_val p_val;
        p_val.x = x;
        p_val.y = y;
        if(valid == true){
            // ロボットの進行方向を x軸とする。
            // + 0.5 は、必要か? 今は、とりあえず加えておく。
            p_val.x_r = ((float)x + 0.5) * res_;    // real [M]
            p_val.y_r = ((float)y + 0.5) * res_;    // real [M]

            // ロボットの進行方向を robo x軸とした場合の、cur plot 位置の角度
            p_val.theta_r = std::atan2(p_val.y_r, p_val.x_r);   //  [radian]
            // test by nishi 2024.12.27
            //p_val.theta_r = std::atan2(p_val.y_r * (-1.0), p_val.x_r);   //  [radian]

            // robot start 位置 の robo x 軸を、 tf-map の x軸と並行にした場合の
            //  cur plot の角度 --> th  tf-x 軸との角度
            float th = theta_r_ + p_val.theta_r;

            // robot start 位置から cur plot までの、距離を求める
            float dist = std::sqrt((float)(p_val.x_r*p_val.x_r + p_val.y_r*p_val.y_r));

            // tf real アドレスを求める [M] origin tf-map(0,0)
            p_val.x_ar = dist * std::cos(th) + sx_;
            p_val.y_ar = dist * std::sin(th) + sy_;

            //std::cout << " x:"<< x <<" y:"<< y << std::endl;
            //std::cout << " p_val.x_r:"<< p_val.x_r <<" p_val.y_r:"<< p_val.y_r << std::endl;
        }
        else{
            p_val.x_r = 0.0;
            p_val.y_r = 0.0;
            p_val.theta_r = 0.0;   //  [radian]
        }
        p_val.valid = valid;
        path_val_.push_back(p_val);
    }

    bool is_all_valid(){
        bool rc=true;
        if(plan_done_ == false)
            return false;
        for(int i = 0; i < path_val_.size(); i++){
            if(path_val_[i].valid == false){
                return false;
            }
        }
        return rc;
    }

    /*
    * path_val_ を圧縮する。
    *   同じ、Path_val.x は纏める。
    */
    bool condense(){
        if(is_all_valid() == false)
            return false;

        if(condense_done == true)
            return true;

        if(path_val_conde_.empty() == false){
            path_val_conde_.clear();
        }
        bool first=true;
        Path_val pv_prev;
        for(Path_val pv: path_val_){
            if(first==true){
                pv_prev=pv;
                first = false;
                continue;
            }
            if(pv_prev.y == pv.y){
                pv_prev=pv;
            }
            else{
                path_val_conde_.push_back(pv_prev);
                pv_prev=pv;
            }
        }
        path_val_conde_.push_back(pv_prev);
        condense_done=true;
        return true;
    }

    bool get_r(float &x_r, float &y_r){
        bool rc_f=true;
        if(cur_p==0){
            if(condense_done == false){
                rc_f=condense();
            }
            if(path_val_conde_.empty() == true)
                return false;
        }
        if(cur_p >= path_val_conde_.size())
            return false;
        // 精度  0.00 [M]
        x_r = round_my<float>(path_val_conde_[cur_p].x_ar,2);
        y_r = round_my<float>(path_val_conde_[cur_p].y_ar,2);
        cur_p++;
        return rc_f;
    }

private:
    bool condense_done;
};

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

    std::mutex mtx_;    // add by nishi 2024.9.28

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