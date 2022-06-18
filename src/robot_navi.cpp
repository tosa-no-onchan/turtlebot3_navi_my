/*
* robot_navi.cpp
*
*
* http://wiki.ros.org/costmap_2d/
*/

#include "turtlebot3_navi_my/robot_navi.h"

void RobotNavi::goalCallback(const geometry_msgs::PoseStamped msg){
  ROS_INFO("recieve");
  last_pose_ = msg;
  nav_state_ = NavState::WAIT_PLAN;
}

/*
* void init()
*/
void RobotNavi::init(ros::NodeHandle &nh,int func){
  nh_=nh;
  func_=func;
  //twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/dtw_robot1/diff_drive_controller/cmd_vel", 10);
  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  // Rviz での Navi操作用の Topic受信
  if(func_==2){
    goal_sub_ = nh_.subscribe("/move_base_simple/goal", 10, &RobotNavi::goalCallback, this);
  }
  global_planner_.initialize("global_planner", &global_costmap_);

#if ROS_VERSION_MINIMUM(1,14,0)
  local_planner_.initialize("local_planner", &tfBuffer, &local_costmap_);
#else
  // here melocic.
  local_planner_.initialize("local_planner", &tf_, &local_costmap_);
#endif

  nav_state_ = NavState::STANDBY;

  if(func_ != 0){
    timer_ = nh_.createTimer(ros::Duration(0.2), &RobotNavi::timerCallback, this);
  }
  // ~<name>/costmap(navi_msgs/OcuupancyGrid) がパブリッシュされ始めれば、セットアップ完了だが、
  // ~<name>: global_costmap or local_costmap
  // global_costmap_,local_costmap_ のメソッドの中で Setup完了チェックできる機能があれば良いのだが?
  // 今は、単純にディレイさせる。
  ros::Rate rate(1);   //  1[Hz]
  //for(int i=0;i<6;i++){
  while(global_costmap_.isCurrent() == false || local_costmap_.isCurrent() == false){
      ros::spinOnce();
      rate.sleep();
  }

  std::cout << "RobotNavi::init() ok!" << std::endl;

}

/*
move()
    x,y: 絶対番地への移動(基準座標)
    r_yaw: 基準座標での角度。 [rad]

  https://answers.ros.org/question/371925/euler-to-quaternion-c/
*/
void RobotNavi::move(float x,float y,float r_yaw){

  std::cout << "RobotNavi::move" << std::endl;

  last_pose_.header.stamp=ros::Time(0);
  // 目標位置
  last_pose_.header.frame_id="map";
  last_pose_.pose.position.x = x;
  last_pose_.pose.position.y = y;
  last_pose_.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0,0.0,r_yaw);
  q=q.normalize();

  last_pose_.pose.orientation.x = q.x();
  last_pose_.pose.orientation.y = q.y();
  last_pose_.pose.orientation.z = q.z();
  last_pose_.pose.orientation.w = q.w();

  if (func_!=0){
    std::cout << "func_ = " << func_ << std::endl;
    nav_state_ = NavState::WAIT_PLAN;

    ros::Rate rate(30);   // 30[Hz] 
    while(nav_state_ != NavState::STANDBY){
      ros::spinOnce();
      rate.sleep();
    }
    return;
  }

  if(MakePlan()==true){
    nav_state_ = NavState::MOVING;

    local_planner_.setPlan(last_global_plan_);
    geometry_msgs::Twist cmd_vel;
    if (local_planner_.isGoalReached()){
      ROS_INFO("reach");
      //twist_pub_.publish(cmd_vel);
      nav_state_ = NavState::STANDBY;
    }
    else{
      local_planner_.computeVelocityCommands(cmd_vel);
      twist_pub_.publish(cmd_vel);
    }

    ros::Rate rate(5.0);   // 5[Hz]  0.2[Sec]

    while(nav_state_ != NavState::STANDBY){
      rate.sleep();

      // ここの判定がシビアすぎか?
      if (local_planner_.isGoalReached()){
        ROS_INFO("reach");
        twist_pub_.publish(cmd_vel);
        nav_state_ = NavState::STANDBY;
        break;
      }
      local_planner_.computeVelocityCommands(cmd_vel);
      twist_pub_.publish(cmd_vel);
    }
  }
}

bool RobotNavi::MakePlan(){
  std::cout << "MakePlan()" << std::endl;

  geometry_msgs::PoseStamped source_pose;
  //source_pose.header.frame_id="dtw_robot1/base_link";
  source_pose.header.frame_id="base_footprint";
  source_pose.header.stamp=ros::Time(0);
  source_pose.pose.orientation.w=1.0;

  geometry_msgs::PoseStamped target_pose;
  //std::string target_frame="dtw_robot1/map";
  std::string target_frame="map";
  try{
    tf_.waitForTransform(source_pose.header.frame_id, target_frame, ros::Time(0), ros::Duration(1.0));
    // 現在位置を得る  -> target_pose
    tf_.transformPose(target_frame, source_pose, target_pose);
    ROS_INFO("x:%+5.2f, y:%+5.2f,z:%+5.2f",target_pose.pose.position.x,target_pose.pose.position.y,target_pose.pose.position.z);
  }
  catch(...){
    ROS_INFO("tf error");
    return false;
  }
  geometry_msgs::PoseStamped start = target_pose;   // 現在位置 / start position

  std::cout << "last_pose_.header.frame_id=" << last_pose_.header.frame_id << std::endl;
  std::cout <<  "last_pose_.header.stamp=" << last_pose_.header.stamp << std::endl;
  std::cout << "last_pose_.pose.position.x,y,z=" << last_pose_.pose.position.x <<","<< last_pose_.pose.position.y <<","<< last_pose_.pose.position.z << std::endl;
  std::cout << "last_pose_.pose.orientation.x,y,z,w=" << last_pose_.pose.orientation.x <<","<< last_pose_.pose.orientation.y 
      <<","<< last_pose_.pose.orientation.z <<","<< last_pose_.pose.orientation.w << std::endl;

  if (!global_planner_.makePlan(start, last_pose_, last_global_plan_)){
    ROS_WARN("global plan fail");
    return false;
  }
  return true;
}


void RobotNavi::timerCallback(const ros::TimerEvent& e){
  if (nav_state_ == NavState::WAIT_PLAN){
    ROS_INFO("PLAN");

    geometry_msgs::PoseStamped source_pose;
    //source_pose.header.frame_id="dtw_robot1/base_link";
    source_pose.header.frame_id="base_footprint";
    source_pose.header.stamp=ros::Time(0);
    source_pose.pose.orientation.w=1.0;

    geometry_msgs::PoseStamped target_pose;
    //std::string target_frame="dtw_robot1/map";
    std::string target_frame="map";
    try{
      tf_.waitForTransform(source_pose.header.frame_id, target_frame, ros::Time(0), ros::Duration(1.0));
      // 現在位置を得る  -> target_pose
      tf_.transformPose(target_frame, source_pose, target_pose);
      ROS_INFO("x:%+5.2f, y:%+5.2f,z:%+5.2f",target_pose.pose.position.x,target_pose.pose.position.y,target_pose.pose.position.z);
    }
    catch(...){
      ROS_INFO("tf error");
    }
    geometry_msgs::PoseStamped start = target_pose;   // 現在位置 / start position
  
    std::cout << "last_pose_.header.frame_id=" << last_pose_.header.frame_id << std::endl;
    std::cout <<  "last_pose_.header.stamp=" << last_pose_.header.stamp << std::endl;
    std::cout << "last_pose_.pose.position.x,y,z=" << last_pose_.pose.position.x <<","<< last_pose_.pose.position.y <<","<< last_pose_.pose.position.z << std::endl;
    std::cout << "last_pose_.pose.orientation.x,y,z,w=" << last_pose_.pose.orientation.x <<","<< last_pose_.pose.orientation.y 
        <<","<< last_pose_.pose.orientation.z <<","<< last_pose_.pose.orientation.w << std::endl;


    if (!global_planner_.makePlan(start, last_pose_, last_global_plan_)){
      ROS_WARN("global plan fail");
      nav_state_ = NavState::STANDBY;
      return;
    }
    local_planner_.setPlan(last_global_plan_);
    geometry_msgs::Twist cmd_vel;
    if (local_planner_.isGoalReached()){
      ROS_INFO("reach");
      twist_pub_.publish(cmd_vel);
      nav_state_ = NavState::STANDBY;
      return;
    }
    local_planner_.computeVelocityCommands(cmd_vel);
    twist_pub_.publish(cmd_vel);
    nav_state_ = NavState::MOVING;
  }
  else if(nav_state_ == NavState::MOVING){
    ROS_INFO_THROTTLE(2.0, "MOVING");
    geometry_msgs::Twist cmd_vel;
    // ここの判定がシビアすぎか?
    if (local_planner_.isGoalReached()){
      ROS_INFO("reach");
      twist_pub_.publish(cmd_vel);
      nav_state_ = NavState::STANDBY;
      return;
    }
    local_planner_.computeVelocityCommands(cmd_vel);
    twist_pub_.publish(cmd_vel);
  }
}

//int main(int argc, char** argv){
//  ros::init(argc, argv, "robot_navigation");
//  
//  RobotNavi robot_navi;
//  ros::spin();
//  return 0;
//}
