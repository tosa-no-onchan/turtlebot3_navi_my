/*
* ROS2 robot_driveNAV2.hpp
*/

#ifndef ROBOT_DRIVE_NAV2_HPP
#define ROBOT_DRIVE_NAV2_HPP

//#include <ros/ros.h>
//#include <geometry_msgs/Twist.h>
//#include <tf/transform_listener.h>


#include "turtlebot3_navi_my/com_lib.hpp"
#include "turtlebot3_navi_my/robot_driveCore.hpp"

#include <functional>
#include <string>

#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/future_return_code.hpp"
#include "rclcpp/executors.hpp"

//#include "rclcpp/client.hpp"
//#include "rclcpp_action/client_goal_handle.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"



//#include "move_base_msgs/msg/"

#define TEST_MOVE_NISI_1

//#include <geometry_msgs/PoseStamped.h>


#ifdef TEST_MOVE_NISI_1
    //#include <move_base_msgs/MoveBaseActionResult.h>
#endif


//#include <tf2/LinearMath/Quaternion.h>
//#include <move_base_msgs/MoveBaseAction.h>
//#include <actionlib/client/simple_action_client.h>

#include "com_def.hpp"

//#include <tf/LinearMath/Matrix3x3.h>


/*
* class RobotDriveNAV2
*   move_base Version
*/
class RobotDriveNAV2:public Robot_DriveCore
{

public:
    //tf2::Stamped<tf2::Transform> base_tf;

    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    //using GoalHandle = ClientGoalHandle<ActionT>;

    //using CancelResponse = typename ActionT::Impl::CancelGoalService::Response;

    using CancelResponse = typename NavigateToPose::Impl::CancelGoalService::Response;
    //using CancelResponseMsg = typename CancelResponse::SharedPtr;


    rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;

    //double _rx, _ry, _rz;

    //bool _course_correct;       // dummy valable
    //bool _after_correct_wait;   // dummy valable
    //bool _go_curve;             // dummy valable
    //bool _dumper;               // dummy valable


    RobotDriveNAV2(){}
    //void init(ros::NodeHandle &nh,bool navi_use=false);
    void init(std::shared_ptr<rclcpp::Node> node,GetTF *getTF,bool navi_use=false);

    void exec_pub(float x,float y,float r_yaw,bool rotate_f=false);
    void cancel_goal(std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>>>goal_handle );

    #ifdef TEST_MOVE_NISI_1
        void cancel_callback(CancelResponse::SharedPtr result);
        //move_base/result (move_base_msgs/MoveBaseActionResult) 
        //void navResultCallback(const move_base_msgs::MoveBaseActionResult msg);
        void navResultCallback(const GoalHandleNavigateToPose::WrappedResult & result);
        void feedbackCallback(GoalHandleNavigateToPose::SharedPtr pose,const std::shared_ptr<const NavigateToPose::Feedback> feedback);

        std::function<void (CancelResponse::SharedPtr)> cancel_callbacks_;

    #endif

    /*
    move()
    自分からの相対位置へ移動
        float dist: 自分からの距離
        float d_yaw: 基準座標での角度。 [degree] ロボット座標上の角度では無い
        bool func_f: false[deafult] d_yaw -> 基準座標での角度(今までの処理)
                     true           d_yaw(+/-) -> ロボットからの角度
    */
    void move(float dist,float d_yaw,bool func_f=false);

    /*
    move_abs()
        x,y: 絶対番地への移動(基準座標)
        d_yaw: 基準座標での角度。 [degree]
    */
    void move_abs(float x,float y,float d_yaw);

    /*
    comp_dad() : compute distanse and direction
    目的地までの距離と方角を計算する。
        float x:
        float y:
        float &dist:
        float &r_yaw:
        float &r_yaw_off:
    */
    void comp_dad(float x,float y,float &dist, float &r_yaw, float &r_yaw_off);


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

    /*
    * void get_tf(int func)
    */
    bool get_tf(int func=0);
    //bool get_tf(int func);

    /*
    go_abs(x,y,isForward=True,speed=0.05)
    直進する。
    */
    void go_abs(float x,float y,float speed=0.05 ,bool isForward=true);

    /*
    rotate_abs()
        stop_dz(d_theta) : [deg] 基本座標上の角度
        rad_f : false -> deg / true -> radian
        speed :  5.0  [deg/s]
    */
    void rotate_abs(float stop_dz,bool rad_f=false, float speed=5.0);
    /* 
    void rotate_off()
        d_theta : [deg] ロボット座標上の角度
        speed :  5.0  [deg/s]
    */
    void rotate_off(float d_theta, float speed=5.0,bool go_curve=false);
    //! Drive forward a specified distance based on odometry information
    bool driveForwardOdom(double distance);
    bool turnOdom(bool clockwise, double radians);

    bool navi_move(float x,float y,float r_yaw,float r_yaw_off=0.0);
    void navi_map_save();

private:
    //! The node handle we'll be using
    //ros::NodeHandle nh_;
    std::shared_ptr<rclcpp::Node> node_;
    //GetTF getTF_;
    GetTF *getTF_; // changed by nishi 2024.2.27

    HeartBeat heartBeat_;   // add by nishi 2023.3.8

    //ros::Publisher pub_;
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> pub_;

    // rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions send_goal_options_;
    //std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>>> goal_handle_

    //! We will be listening to TF transforms as well
    //tf::TransformListener listener_;

    //ros::Subscriber goal_sub_;
    //rclcpp::Subscription<move_base_msgs::MoveBaseActionResult>::SharedPtr goal_sub_;


    u_char log_level=1;

    //int id_=0;      // 0: idle  1: navigate 2:arrive  4>=:error

    enum drive_sts {
        // ---- loop status
        NAV_IDLE,       // 0
        NAV_NAVIGATE,   // 1
        NAV_CANCELD_REQ,
        NAV_TF_WAIT,
        // ---- exit status
        NAV_ARRIVE,     // 2
        NAV_ERROR,      // 3
        NAV_ABORTED,    // 4
        NAV_CANCELD,    // 9
        NAV_UNKNOWN,    // 9
        NAV_ERROR10    // 10
    };
    drive_sts id_;

    //bool rotate_f;

};

#endif      // ROBOT_DRIVE_MB_HPP


