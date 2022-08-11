#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>


#define TEST_MOVE_NISI_1
//#define TEST_MOVE_NISI_2

#include <geometry_msgs/PoseStamped.h>


#ifdef TEST_MOVE_NISI_1
    #include <move_base_msgs/MoveBaseActionResult.h>
#endif

#ifdef TEST_MOVE_NISI_2
    #include <actionlib_msgs/GoalStatusArray.h>
#endif

//#include <actionlib_msgs/GoalStatusArray.h>

#include <tf2/LinearMath/Quaternion.h>

//#include <ros/ros.h>

#include <move_base_msgs/MoveBaseAction.h>

#include <actionlib/client/simple_action_client.h>

#include "com_def.h"

#include <tf/LinearMath/Matrix3x3.h>

/*
* class RobotDriveMB
*   move_base Version
*/

class RobotDriveMB{
private:
    //! The node handle we'll be using
    ros::NodeHandle nh_;

    ros::Publisher pub_;
    //! We will be listening to TF transforms as well
    tf::TransformListener listener_;

    #ifdef TEST_MOVE_NISI_1
        ros::Subscriber goal_sub_;
    #endif

    #ifdef TEST_MOVE_NISI_2
        ros::Subscriber goal_sub2_;
    #endif

    u_char log_level=1;

    int id_=0;      // 0: idle  1: navigate 2:arrive  4>=:error

    //bool rotate_f;


public:
    tf::StampedTransform base_tf;

    double _rx, _ry, _rz;

    bool _course_correct;       // dummy valable
    bool _after_correct_wait;   // dummy valable
    bool _go_curve;             // dummy valable
    bool _dumper;               // dummy valable


    RobotDriveMB(){}
    void init(ros::NodeHandle &nh,bool navi_use=false);

    void exec_pub(float x,float y,float r_yaw,bool rotate_f=false);

    #ifdef TEST_MOVE_NISI_1
        //move_base/result (move_base_msgs/MoveBaseActionResult) 
        void goalCallback(const move_base_msgs::MoveBaseActionResult msg);
    #endif
    #ifdef TEST_MOVE_NISI_2
        // move_base/status (actionlib_msgs/GoalStatusArray) 
        void navStatusCallBack(const actionlib_msgs::GoalStatusArray::ConstPtr &status);
    #endif


    /*
    move()
    自分からの相対位置へ移動
        dist: 自分からの距離
        d_yaw: 基準座標での角度。 [degree] ロボット座標上の角度では無い
    */
    void move(float dist,float d_yaw);

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
    void get_tf(int func=0);

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

};


