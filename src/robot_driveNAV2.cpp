/*
robot_driveNAV2.cpp
robot_drivee for move__base

https://wiki.ros.org/pr2_controllers/Tutorials/Using%20the%20base%20controller%20with%20odometry%20and%20transform%20information

https://www.k-cube.co.jp/wakaba/server/func/math_h.html

https://answers.ros.org/question/50113/transform-quaternion/

build
$ catkin_make --pkg turtlebot3_navi_my

$ rosrun turtlebot3_navi_my drive_base
*/

#include "turtlebot3_navi_my/robot_driveNAV2.hpp"

#include <future>

using std::placeholders::_1;
using std::placeholders::_2;


//---------------------------------
// cancel
//---------------------------------
void RobotDriveNAV2::cancel_callback(CancelResponse::SharedPtr result){
    std::cout << "RobotDriveNAV2::cancel_callback() called!!" << std::endl;
}

//---------------------------------
// result
//---------------------------------
//void RobotDriveNAV2::navResultCallback(const move_base_msgs::MoveBaseActionResult msg){
void RobotDriveNAV2::navResultCallback(const GoalHandleNavigateToPose::WrappedResult & result){

    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(node_->get_logger(), "Success!!!");
        id_=2;
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(node_->get_logger(), "Goal was aborted");
        id_=4;
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(node_->get_logger(), "Goal was canceled");
        id_=9;
        return;
      default:
        RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
        id_=9;
        return;
    }
}

//------------------------------
// feedback
//------------------------------
//void RobotDriveNAV2::feedbackCallback(GoalHandleNavigateToPose::SharedPtr,const std::shared_ptr<const NavigateToPose::Feedback> feedback){
void RobotDriveNAV2::feedbackCallback(GoalHandleNavigateToPose::SharedPtr pose,const std::shared_ptr<const NavigateToPose::Feedback> feedback){
    //RCLCPP_INFO(node_->get_logger(), "Distance remaininf = %f", feedback->distance_remaining);
}


/*
* void get_tf(int func)
*/
void RobotDriveNAV2::get_tf(int func){
    getTF_.get(func);

    base_tf=getTF_.base_tf;

    if (func==2){
        _rx=getTF_._rx;
        _ry=getTF_._ry;
        _rz=getTF_._rz;
        if(log_level>=3)
            std::cout << "_rx: " << _rx << ", _ry: " << _ry << ", _rz: " << _rz << std::endl;
    }
}

//void RobotDriveNAV2::init(ros::NodeHandle &nh,bool navi_use){
void RobotDriveNAV2::init(std::shared_ptr<rclcpp::Node> node,bool navi_use){
    node_=node;

    rclcpp::WallRate rate(1);
    std::cout << "RobotDriveNAV2::init():#1 " << std::endl;

    getTF_.init(node_);

    std::cout << "RobotDriveNAV2::init():#2 " << std::endl;

    //アクション Client の作成
    this->client_ptr_  = rclcpp_action::create_client<NavigateToPose,std::shared_ptr<rclcpp::Node>>(node_, "navigate_to_pose");
    rate.sleep();

    std::cout << "RobotDriveNAV2::init():#3 " << std::endl;

    // アクションが提供されているまでに待つ
    //while (!this->client_ptr_->wait_for_action_server()) {
    while (!this->client_ptr_->action_server_is_ready()) {
      RCLCPP_INFO(node_->get_logger(), "Waiting for action server...");
      rate.sleep();
      rclcpp::spin_some(node_);
    }

    std::cout << "RobotDriveNAV2::init():#4 " << std::endl;

    //pub_ = nh_.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
    //pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("move_base_simple/goal", 1);

    //goal_sub_ = nh_.subscribe("move_base/result", 10, &RobotDriveNAV2::navResultCallback, this);
    //goal_sub_ = node_->create_subscription<std_msgs::msg::String>(
    //  "move_base/result", 10, std::bind(&RobotDriveNAV2::navResultCallback, this, _1));
    //move_base/result (move_base_msgs/MoveBaseActionResult) 


    //listener_.waitForTransform("map","base_footprint", ros::Time(0), ros::Duration(1.0));
    //sleep(1);
    rate.sleep();

    //進捗状況を表示するFeedbackコールバックを設定
    // rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions
    send_goal_options_ = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options_.feedback_callback = std::bind(&RobotDriveNAV2::feedbackCallback, this, _1, _2);
    send_goal_options_.result_callback = std::bind(&RobotDriveNAV2::navResultCallback, this, _1);

    cancel_callbacks_ =std::bind(&RobotDriveNAV2::cancel_callback, this, _1);

    std::cout << "RobotDriveNAV2::init():#5 " << std::endl;

    rate.sleep();

}

void RobotDriveNAV2::exec_pub(float x,float y,float r_yaw,bool rotate_f){

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, r_yaw);        // q.setRPY( 0, 0, 0 );  // Create this quaternion from roll/pitch/yaw (in radians)
    q=q.normalize();


    std::chrono::milliseconds server_timeout_(10);

    //std::cout << "x: " << q.getX() << " y: " << q.getY() << 
    //                " z: " << q.getZ() << " w: " << q.getW() << std::endl;

    //ros::Rate one_sec(1);
    //rclcpp::WallRate one_sec(1);
    //one_sec.sleep();
     
    //ros::Time time = ros::Time::now();
    rclcpp::Time time = node_->now();

    //geometry_msgs::PoseStamped goal_point;
    //アクション　Goalの作成
    auto goal_point = NavigateToPose::Goal();
    //nav2_msgs::action::NavigateToPose_Goal goal_point = NavigateToPose::Goal();
    // nav2_msgs::action::NavigateToPose_Goal_
    //printf("%s",goal_point);
 
    goal_point.pose.pose.position.x = x;
    goal_point.pose.pose.position.y = y;
    goal_point.pose.pose.position.z =  0.0;
    goal_point.pose.pose.orientation.x = q.getX();
    goal_point.pose.pose.orientation.y = q.getY();
    goal_point.pose.pose.orientation.z = q.getZ();
    goal_point.pose.pose.orientation.w = q.getW();
    goal_point.pose.header.stamp = time;
    goal_point.pose.header.frame_id = "map";
 
    // samples are 
    // navigation2/nav2_util/test/test_actions.cpp
    // navigation2/nav2_rviz_plugins/src/nav2_pnale.cpp

    //Goal をサーバーに送信
    auto goal_handle_ = client_ptr_->async_send_goal(goal_point, send_goal_options_);
    //std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>>> goal_handle_ = client_ptr_->async_send_goal(goal_point, send_goal_options_);
    // std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose> > >
    //printf("%s",goal_handle);

    if(rclcpp::spin_until_future_complete(node_,goal_handle_) != rclcpp::FutureReturnCode::SUCCESS){
        RCLCPP_ERROR(node_->get_logger(), "Send goal call failed");
        return;
    }


    //ros::Rate rate(50.0); // [Hz]
    rclcpp::WallRate rate(50.0);

    //pub_.publish(goal_point);
    id_=1;

    int i=0;
    int moving_cnt =0;
    int moving_err_cnt=0;

    float last_x=0.0;
    float last_y=0.0;

    while(1){
        rate.sleep();
        //ros::spinOnce();
        rclcpp::spin_some(node_);

        if(id_ >= 2)
            break;

        moving_cnt ++;
        if(rotate_f==false){
            if(moving_cnt > 150){
                get_tf();
                //tf::Vector3 cur_origin = base_tf.getOrigin();
                tf2::Vector3 cur_origin = base_tf.getOrigin();
                float x = cur_origin.getX();
                float y = cur_origin.getY();

                float off_x = x -  last_x;
                float off_y = y -  last_y;

                float cur_dist = std::sqrt(off_x*off_x+off_y*off_y);

                last_x=x;
                last_y=y;

                cur_dist = round_my<float>(cur_dist,3);
                //if(cur_dist <= 0.005){
                //if(cur_dist <= 0.01){
                //if(cur_dist <= 0.03){       // changed by nishi 2022.8.17
                if(cur_dist <= 0.04){       // changed by nishi 2022.8.17
                    moving_err_cnt++;
                    std::cout << "not moving (" << moving_err_cnt << ")" << std::endl;
                }
                else{
                    moving_err_cnt=0;
                    std::cout << "moving" << std::endl;
                }

                if(moving_err_cnt >= 16){
                    std::cout << "not moving exceed limits" << std::endl;
                    /*
                    * cancel
                    *   async_cancel_goal(
                    *       typename GoalHandle::SharedPtr goal_handle,
                    *       CancelCallback cancel_callback = nullptr)
                    *       client_ptr_->async_cancel_goal(goal_handle_);
                    *
                    * Sample
                    *  navigation2/nav2_util/test/test_actions.cpp
                    */
                    std::cout << "call client_ptr_->async_cancel_goal()" << std::endl;
                    //auto cancel_response = client_ptr_->async_cancel_goal(goal_handle_.get());
                    auto cancel_response = client_ptr_->async_cancel_goal(goal_handle_.get(),cancel_callbacks_);
                    if (rclcpp::spin_until_future_complete(node_, cancel_response, server_timeout_) != rclcpp::FutureReturnCode::SUCCESS)
                    {
                        RCLCPP_ERROR(node_->get_logger(), "Failed to cancel async_cancel_goal()");
                        //return;
                        //std::cout << "client_ptr_->async_cancel_goal() OK" << std::endl;
                    }
                    // Check cancelled
                    if (goal_handle_.get()->get_status() == rclcpp_action::GoalStatus::STATUS_CANCELING){
                        std::cout << "cancelling OK" << std::endl;
                    }
                    else{
                        std::cout << "cancelling NG" << std::endl;
                    }
                    id_=10;
                }

                moving_cnt=0;
            }
        }
        else{
            if((moving_cnt % 150) == 0){
                std::cout << "rotate wait"<< std::endl;
            }
            //if(moving_cnt > 1500){    // 10 loops
            if(moving_cnt > 2400){      // 16 loops changed by nishi 2022.9.4
                std::cout << "move_base exceed rotate count"<< std::endl;
                // cnacel 
                //auto cancel_callbacks =std::bind(&RobotDriveNAV2::cancel_callback, this, _1);
                auto cancel_response = client_ptr_->async_cancel_goal(goal_handle_.get(),cancel_callbacks_);
                if (rclcpp::spin_until_future_complete(node_, cancel_response, server_timeout_) != rclcpp::FutureReturnCode::SUCCESS)
                {
                    RCLCPP_ERROR(node_->get_logger(), "Failed to cancel async_cancel_goal()");
                    //return;
                    //std::cout << "client_ptr_->async_cancel_goal() OK" << std::endl;
                }
                // Check cancelled
                if (goal_handle_.get()->get_status() == rclcpp_action::GoalStatus::STATUS_CANCELING){
                    std::cout << "cancelling OK" << std::endl;
                }
                else{
                    std::cout << "cancelling NG" << std::endl;
                }

                id_=10;
            }
        }
    }
    if(id_==2){
        std::cout << "move_base arrive"<< std::endl;
    }
    else if(id_>= 3){
        std::cout << "move_base error"<< std::endl;
    }

}

/*
move()
自分からの相対位置へ移動
    dist: 自分からの距離
    d_yaw: 基準座標での角度。 [degree] ロボット座標上の角度では無い
*/
void RobotDriveNAV2::move(float dist,float d_yaw){
    get_tf();
    //tf::Vector3 start_origin = base_tf.getOrigin();
    tf2::Vector3 start_origin = base_tf.getOrigin();

    float start_x = start_origin.getX();
    float start_y = start_origin.getY();

    float r_yaw = d_yaw/RADIANS_F;
    // 目的地を計算
    //float y = self.base_tf.transform.translation.y + dist * math.sin(r_yaw);
    float y = start_y + dist * std::sin(r_yaw);
    //float x = self.base_tf.transform.translation.x + dist * math.cos(r_yaw);
    float x = start_x + dist * std::cos(r_yaw);
    rotate_abs(d_yaw);
    if (dist != 0.0)
        go_abs(x,y);
}

/*
move_abs()
    x,y: 絶対番地への移動(基準座標)
    d_yaw: 基準座標での角度。 [degree]
*/
void RobotDriveNAV2::move_abs(float x,float y,float d_yaw){
    std::cout << "RobotDriveNAV2::move_abs() called"<< std::endl;
    std::cout << "x:"<< x << " y:" << y << " d_yaw:" << d_yaw << std::endl;
    //rotate_f=false;

    float r_yaw=d_yaw/RADIANS_F;

    exec_pub(x,y,r_yaw);

}

/*
comp_dad() : compute distanse and direction
 目的地までの距離と方角を計算する。
    float x:
    float y:
    float &dist:
    float &r_yaw:
*/
void RobotDriveNAV2::comp_dad(float x,float y,float &dist, float &r_yaw, float &r_yaw_off){
    get_tf(1);

    dist=0.0;
    r_yaw=0.0;
    r_yaw_off=0.0;

    //tf::Vector3 cur_origin = base_tf.getOrigin();
    tf2::Vector3 cur_origin = base_tf.getOrigin();

    float cur_x = cur_origin.getX();
    float cur_y = cur_origin.getY();

    float off_x = x - cur_x;
    float off_y = y - cur_y;

    float cur_dist = std::sqrt(off_x*off_x+off_y*off_y);
    cur_dist = round_my<float>(cur_dist,3);

    if(cur_dist > 0.0){
        float r_theta = std::atan2(off_y,off_x);   //  [ragian]
        // ロボットの位置の原点からの向きは、
        float r_theta_robo = std::atan2(cur_y,cur_x);

        dist=cur_dist;
        //r_yaw= r_theta+r_theta_robo;
        r_yaw= r_theta;
        r_yaw_off = r_theta;
    }
}

/*
go_abs(x,y,isForward=True,speed=0.05)
直進する。
*/
void RobotDriveNAV2::go_abs(float x,float y,float speed ,bool isForward){

    std::cout << "go_abs" << std::endl;

}

/*
rotate_abs()
    stop_dz(d_theta) : [deg] 基本座標上の角度
    rad_f : false -> deg / true -> radian
    speed :  5.0  [deg/s]
*/
void RobotDriveNAV2::rotate_abs(float stop_dz,bool rad_f,float speed){
    // 目的の角度と速度を設定
    // stop_dz = 180.0 # [deg]
    // speed = 10.0 # [deg/s]

    std::cout << "rotate_abs()" << std::endl;
    if(rad_f==false){
        std::cout << "stop_dz=" << stop_dz << std::endl;
    }
    else{
        std::cout << "stop_dz=" << stop_dz*RADIANS_F << std::endl;
    }
    get_tf();
    //tf::Vector3 start_origin = base_tf.getOrigin();
    tf2::Vector3 start_origin = base_tf.getOrigin();

    float start_x = start_origin.getX();
    float start_y = start_origin.getY();

    float r_yaw;
    if(rad_f==false){
        r_yaw = stop_dz/RADIANS_F;
    }
    else{
        r_yaw = stop_dz;
    }
    //rotate_f=true;

    exec_pub(start_x,start_y,r_yaw,true);

}

/* 
void rotate_off()
    d_theta : [deg] ロボット座標上の角度
    speed :  5.0  [deg/s]

    クォータニオン(四元数)を使用して座標を回転させる
    https://www.kazetest.com/vcmemo/quaternion-rotation/quaternion-rotation.htm

    単純に、オイラー角 で、z軸加算して、クォータニオン に戻す。

*/
void RobotDriveNAV2::rotate_off(float d_theta, float speed, bool go_curve){
    // 目的の角度と速度を設定
    //d_theta = 180.0 # [deg]
    //speed = 10.0 # [deg/s]
    std::cout << "RobotDriveNAV2::rotate_off() called"<< std::endl;
    std::cout << "d_theta:" << d_theta << std::endl;

    float r_theta = d_theta/RADIANS_F;

    get_tf(2);      // _rx, _ry, _rz

    //tf::Vector3 start_origin = base_tf.getOrigin();
    tf2::Vector3 start_origin = base_tf.getOrigin();

    float start_x = start_origin.getX();
    float start_y = start_origin.getY();

    //rotate_f=true;

    exec_pub(start_x,start_y,r_theta+_rz,true);

}

//! Drive forward a specified distance based on odometry information
bool RobotDriveNAV2::driveForwardOdom(double distance)
{
  

  return false;
}

bool RobotDriveNAV2::turnOdom(bool clockwise, double radians)
{

  return false;
}


/*-----------------------
- robot_navi call routine
    robot_drive.cpp とインターフェース合わせ
    の為作成
--------------------------*/
/*
navi_move()
    x,y: 絶対番地への移動(基準座標)
    r_yaw: 基準座標での角度。 [rad]
*/
///void RobotDriveNAV2::navi_move(float x,float y,float r_yaw){
bool RobotDriveNAV2::navi_move(float x,float y,float r_yaw,float r_yaw_off){
    rotate_abs(r_yaw,true);
    exec_pub(x,y,r_yaw);
    if(id_ >=3)
    {
        return false;
    }

    return true;
}

/*
navi_map_save()
 Dummy
*/
void RobotDriveNAV2::navi_map_save(){
    //navi_.map_save();
}