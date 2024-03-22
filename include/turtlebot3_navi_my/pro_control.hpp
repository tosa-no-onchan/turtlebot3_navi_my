/*
* Programable Controller Core
*  turtlebot3_navi_my/include/turtlebot3_navi_my/pro_control.hpp
*
* pro_control.hpp
*
*/

#ifndef PRO_CONTROL_HPP
#define PRO_CONTROL_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>


#include <math.h>
#include <cstdio>
#include <vector>
#include <list>
#include <algorithm>
#include <functional>

//#include "ros/console.h"
//#include "nav_msgs/GetMap.h"
//#include <tf/LinearMath/Matrix3x3.h>
//#include <geometry_msgs/Quaternion.h>


#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/transform_datatypes.h"


#include <iostream>

//#include <ros/ros.h>
//#include <ros/topic.h>
//#include <geometry_msgs/Twist.h>
//#include <tf/transform_listener.h>
//#include "std_srvs/Empty.h"
#include <unistd.h>

//#include <math.h>

#if defined(USE_NAV2)
    #include "turtlebot3_navi_my/robot_driveNAV2.hpp"
    #include "turtlebot3_navi_my/robot_driveCmd_Vel.hpp"
#else
    #include "turtlebot3_navi_my/robot_driveCmd_Vel.hpp"
#endif

//#include <nav_msgs/OccupancyGrid.h>
//#include <nav_msgs/msg/OccupancyGrid.h>

#include "nav_msgs/msg/map_meta_data.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "com_def.hpp"
#include "com_lib.hpp"

#include <opencv2/opencv.hpp>

#include "pro_control_sub.hpp"

// mulit_goals_sub.hpp の方で、定義します。
#if defined(DEFINE_THIS_PARAM)
//#define USE_MAP_SAVER 
// map_server/map_saver の画像を使う時は、こちらを使って下さい。
#ifdef USE_MAP_SAVER
    #define FREE_AREA 254
    #define UNKNOWN_AREA 255

// オンちゃん独自の  保管画像を使う時は、こちら
#else
    #define FREE_AREA 0xff
    #define UNKNOWN_AREA 0x80
#endif

#define COLOR_1 40

#define G_POINTS_MAX 70
#define G_POINTS_MAX1_2 50
#endif

/*----------------------------
- Programable Controller Core Class
-   class Programable Controller
----------------------------*/
class ProControl
{
private:
    int sts;
    int goalId;

    u_int8_t t_type;

    //! The node handle we'll be using
    //ros::NodeHandle nh_;
    std::shared_ptr<rclcpp::Node> node_;

    GoalList *_goalList;
    GoalList2 *_goalList2;

    bool force_start_origin;
    float start_pos_x;
    float start_pos_y;
    float start_pos_z;
    float start_rot_rz;   // start 時の tf-map への オフセット 角度 add by nishi 2022.11.15

    tf2::Vector3 cur_pos;


    int get_map_func_=0;

    // definition for RobotDrive 2024.2.29
    int mode_f;   // 1: navigation2 mode 0:vmd_vel mode
    int mode_f_origin;

public:

    GetTF getTF; // add by nishi 2024.2.27
    Robot_DriveCore *drive_;
    #if defined(USE_NAV2)
        RobotDriveNAV2 drive_nav;   // navigation2
        RobotDriveCmd_Vel drive_cmd;   // cmd_vel
        //#define drive_cmd drive_nav
    #else
        RobotDriveCmd_Vel drive_cmd;       // cmd_vel
    #endif


    GetMap get_map;

    ProControl(){}

    //void init( map_frame,get_map,use_sim_time){
    //void init(ros::NodeHandle &nh);
    void init(std::shared_ptr<rclcpp::Node> node);


    /*
    mloop_ex(GoalList *goalList)
        goalList: ゴールリスト
    */
    void mloop_ex(GoalList *goalList);
    /*
    mloop_ex2(GoalList2 *goalList2)
        goalList: ゴールリスト2
    */
    void mloop_ex2(GoalList2 *goalList2);

    /*
    * set_drive_mode(int func)
    * func: 0 -> cmd_vel, 1 -> navi2
    */
    void set_drive_mode(int func);

    void check_obstacle_backaround(float r_lng=0.60,int black_thresh=8);

    void obstacle_escape(float r_lng=0.60,int black_thresh=0,float move_l=0.12);

    // child class で使わない時は、ダミーを定義して下さい。
    // Auto map 向けのメンバー
    virtual void auto_map(){};
    virtual void auto_map_anchor(){};
    virtual void set_border_top_right(float x, float y){};
    virtual void set_border_bottom_left(float x, float y){};
    virtual void set_line_w(float x){};
    virtual void set_robo_r(float x){};

    // Auto mower 向けのメンバー
    virtual void auto_mower(){};


    /*
    mloop(self)
      self.goalList =[[func,x,y,d_yaw],....] or [[func,dist,d_yaw],....]
        func,x,y,d_yaw
            func: 0 -> move point x,y, and rotate d_yaw
                1 -> move point x,y only
                2 -> rotate d_yaw only
                10 -> navi move x,y,d_yaw

        func,dist,d_yaw
            func: 0 -> move dist and rotate d_yaw

        func
            21 -> sleep
            22 -> get map
            23 -> map update
            30 -> auto map build
            31 -> auto map build of anchor
            35 -> auto mower
            50 -> set Navigation mode
            60 -> course_correct ON
            61 -> course_correct OFF
            62 -> after_correct_wait ON
            63 -> after_correct_wait OFF
            64 -> go curve ON
            65 -> go curve OFF
            66 -> set current postion as map(0,0)
            67 -> set dumper ON
            68 -> set dumper OFF
            69 -> save local cost map
            70 -> set border top-right
            71 -> set border bottom-left
            72 -> set line_w_
            73 -> set robo_r_
            80 -> navigation2 mode
            81 -> cmd_vel mode
            99 -> end

    */
    void mloop();

    void mloop_sub();

    #ifdef KKKKK_1
    void get_odom(){
        odom_msg=None;
        int cnt=30;
        float x=None;
        float y=None;
        float z=None;
        float ox=None;
        float oy=None;
        float oz=None;

        while (odom_msg is None and cnt >=0){
            try:
                if (self.use_sim_time == true)
                    odom_msg = rospy.wait_for_message('/odom', Odometry, timeout=5);
                else
                    odom_msg = rospy.wait_for_message('/odom_fox', Odometry, timeout=5);
            except:
                pass
            cnt-=1;
        }
        if (odom_msg != None){
            // x[M] ,y[M]
            x =self.goalMsg.pose.position.x - odom_msg.pose.pose.position.x;
            y =self.goalMsg.pose.position.y - odom_msg.pose.pose.position.y;
            z =self.goalMsg.pose.position.z - odom_msg.pose.pose.position.z;

            //ox =round(self.goalMsg.pose.orientation.x - odom_msg.pose.pose.orientation.x,4)
            //oy =round(self.goalMsg.pose.orientation.y - odom_msg.pose.pose.orientation.y,4)
            //oz =round(self.goalMsg.pose.orientation.z - odom_msg.pose.pose.orientation.z,4)
            //ow =round(self.goalMsg.pose.orientation.w - odom_msg.pose.pose.orientation.w,4)

            pe = tf.transformations.euler_from_quaternion((self.goalMsg.pose.orientation.x, self.goalMsg.pose.orientation.y, self.goalMsg.pose.orientation.z, self.goalMsg.pose.orientation.w));
            //print 'pe[0],pe[1],pe[2]=',pe[0],pe[1],pe[2]
            oe = tf.transformations.euler_from_quaternion((odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w));
            ox = pe[0]-oe[0];
            oy = pe[1]-oe[1];
            oz = pe[2]-oe[2];

            print 'differ x,y,z:ox,oy,oz =',round(x,4),round(y,4),round(z,4),':',round(ox,4),round(oy,4),round(oz,4)
            print 'ok'
        }
        else
            print 'error';
        return x,y,z,ox,oy,oz;
    }
    #endif

    /*
    * https://qiita.com/hoshianaaa/items/74b0ffbcbf97f4938a4d
    * http://forestofazumino.web.fc2.com/ros/ros_service.html
    */
    void call_service();

};


#endif      // MULTI_GOALS_H
