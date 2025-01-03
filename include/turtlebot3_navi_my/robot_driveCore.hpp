/*
* robot_driveCore.hpp
*
*
*/

#ifndef ROBOT_DRIVE_CORE_HPP
#define ROBOT_DRIVE_CORE_HPP

#include <iostream>
#include "turtlebot3_navi_my/com_lib.hpp"

class Robot_DriveCore
{

public:
    tf2::Stamped<tf2::Transform> base_tf;

    double _rx, _ry, _rz;

    bool _course_correct;       // dummy valable
    bool _after_correct_wait;   // dummy valable
    bool _go_curve;             // dummy valable
    bool _dumper;               // dummy valable

    // add by nishi 2024.9.7
    //float rotate_speed_min_ = 3.0;
    float rotate_speed_min_ = 2.0;
    int rotate_lp_cnt_chk_2_ = 20*2;      //  for time out 2

    float robo_radian_marker_=0.2;      // add by nishi 2024.9.25
    // get_costmap_->cource_obstacle_eye() start disatnace
    float obstacle_eye_start_= 0.25;  // add by nishi 2024.9.27
    // get_costmap_->cource_obstacle_eye() stop disatnace
    //  obstacle_eye_dist_+n
    float obstacle_eye_stop_= 0.25; // add by nishi 2024.9.27
    // get_costmap_->cource_obstacle_eye() check disatnace
    float obstacle_eye_range_ = 0.4; // add by nishi 2024.9.27

    //! ROS node initialization
    Robot_DriveCore(){}
    virtual void init(std::shared_ptr<rclcpp::Node> node,GetTF *getTF,bool navi_use=false){};

    /*
    move()
    自分からの相対位置へ移動
        float dist: 自分からの距離
                    > 0 前進
                    < 0 後退
        float d_yaw: ロボットからの角度。 [degree] 
    */
    virtual void move(float dist,float d_yaw){};

    /*
    move_abs()
        x,y: 絶対番地への移動(基準座標)
        d_yaw: 基準座標での角度。 [degree]
    */
    virtual int move_abs(float x,float y,float d_yaw){return 0;};

    /*
    comp_dad() : compute distanse and direction
    目的地までの距離と方角を計算する。
        float x:
        float y:
        float &dist:
        float &r_yaw:
        float &r_yaw_off:
    */
    virtual void comp_dad(float x,float y,float &dist, float &r_yaw, float &r_yaw_off){};

    /*
    * bool get_tf(int func)
    */
    virtual bool get_tf(int func=0){return true;};

    /*
    int go_path(Path_plan *path_plan, bool obs_chk=true, float speed=0.05)
    path_plan に従った走行をする。
    */
    virtual int go_path(Path_plan *path_plan, bool obs_chk=true, float speed=0.05){return 0;};

    /*
    go_abs(x,y,isBack=falsse,speed=0.05)
    直進する。
    */
    //virtual void go_abs(float x,float y,bool isBack=false,float speed=0.05 ){};
    virtual int go_abs(float x,float y,bool isBack=false, bool obs_chk=false, float speed=0.05){return 0;};

    /*
    robot_driveCmd_vel
    rotate_abs()
        stop_dz(d_theta) : [deg] 基本座標上の角度  > 0 左回転 /  < 0 右回転
        speed : 8.0  [deg/s]
    */
    //virtual void rotate_abs(float stop_dz,float speed=8.0){};

    /*
    robot_driveNAV2
    rotate_abs()
        stop_dz(d_theta) : [deg] 基本座標上の角度  > 0 左回転 /  < 0 右回転
        rad_f : false -> deg / true -> radian
        speed :  15.0  [deg/s]
    */
    virtual void rotate_abs(float stop_dz,bool rad_f=false, float speed=15.0){};


    /* 
    void rotate_off()
        d_theta : [deg] ロボットの今の向きからの角度   > 0 左回転 /  < 0 右回転
        speed :  15.0  [deg/s]
    */
    virtual void rotate_off(float d_theta, float speed=15.0,bool go_curve=false){};

    virtual bool navi_move(float x,float y,float r_yaw,float r_yaw_off=0.0){return true;};

    virtual void navi_map_save(){};

    void set_robo_radian_marker_etc(float robo_radian_marker, float obstacle_eye_start, float obstacle_eye_stop, float obstacle_eye_range){
        robo_radian_marker_=robo_radian_marker;
        obstacle_eye_start_=obstacle_eye_start;
        obstacle_eye_stop_=obstacle_eye_stop;
        obstacle_eye_range_=obstacle_eye_range;
    }




private:

};




#endif
