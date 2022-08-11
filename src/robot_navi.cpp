/*
* robot_navi.cpp
*
*
* http://wiki.ros.org/costmap_2d/
*
* http://docs.ros.org/en/jade/api/costmap_2d/html/classcostmap__2d_1_1Costmap2DROS.html
* http://docs.ros.org/en/jade/api/costmap_2d/html/namespacecostmap__2d.html
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

  ros::Rate rate(5);   //  5[Hz]

  //twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/dtw_robot1/diff_drive_controller/cmd_vel", 10);
  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  rate.sleep();
  ros::spinOnce();

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

  // local cost map access add by nishi 2022.7.7
  costmap2D_= local_costmap_.getCostmap();
  footprint_spec_ = local_costmap_.getRobotFootprint();


  //for(int i=0;i<6;i++){
  while(global_costmap_.isCurrent() == false || local_costmap_.isCurrent() == false){
      ros::spinOnce();
      rate.sleep();
  }


  std::cout << "RobotNavi::init() ok!" << std::endl;

}

void RobotNavi::map_save(){
  //local_costmap_.updateMap();
  //costmap2D_= local_costmap_.getCostmap();
  std::cout << "save map start" << std::endl;
  costmap2D_->saveMap("/home/nishi/local_costmap.pgm");
  std::cout << "save map end" << std::endl;
}

/*
* unsigned char RobotNavi::check_cost(float x,float y)
*/
unsigned char RobotNavi::check_cost(float x,float y){
  std::cout << "RobotNavi::check_cost() start" << std::endl;

  //ros::spinOnce();  // 必要みたい。


  //costmap2D_= local_costmap_.getCostmap();
  //footprint_spec_ = local_costmap_.getRobotFootprint();

  /*
  double 	getOriginX () const
    Accessor for the x origin of the costmap.
  double 	getOriginY () const
    Accessor for the y origin of the costmap.
  double 	getResolution () const
    Accessor for the resolution of the costmap.
  unsigned int 	getSizeInCellsX () const
    Accessor for the x size of the costmap in cells.
  unsigned int 	getSizeInCellsY () const
    Accessor for the y size of the costmap in cells.
  double 	getSizeInMetersX () const
    Accessor for the x size of the costmap in meters.
  double 	getSizeInMetersY () const 
  */

  /*
  std::cout << "ostmap2D_->getOriginX()=" << costmap2D_->getOriginX() << std::endl;
  std::cout << "ostmap2D_->getOriginY()=" << costmap2D_->getOriginX() << std::endl;
  std::cout << "ostmap2D_->getResolution()=" << costmap2D_->getResolution() << std::endl;
  std::cout << "ostmap2D_->getSizeInCellsX()=" << costmap2D_->getSizeInCellsX() << std::endl;
  std::cout << "ostmap2D_->getSizeInCellsY()=" << costmap2D_->getSizeInCellsY() << std::endl;

  std::cout << "ostmap2D_->getSizeInMetersX()=" << costmap2D_->getSizeInMetersX() << std::endl;
  std::cout << "ostmap2D_->getSizeInMetersY()=" << costmap2D_->getSizeInMetersY() << std::endl;
  */

  /*
  ostmap2D_->getOriginX()=-4.95
  ostmap2D_->getOriginY()=-4.95
  ostmap2D_->getResolution()=0.05
  ostmap2D_->getSizeInCellsX()=80
  ostmap2D_->getSizeInCellsY()=80
  ostmap2D_->getSizeInMetersX()=3.975
  ostmap2D_->getSizeInMetersY()=3.975
  */


  geometry_msgs::Point p;
  unsigned int mx,my;
  unsigned char cost=0;
  for(int i=0;i<footprint_spec_.size();i++){
      p = footprint_spec_[i];
      //std::cout <<"x,y,z = "<< p.x <<","<< p.y <<"," << p.z << std::endl;
      if(costmap2D_->worldToMap(p.x+x, p.y+y, mx,my)==true){
          //costmap2D_->worldToMapEnforceBounds(p.x+x, p.y+y, mx,my);
          //std::cout <<"mx,my = "<< mx <<","<< my << std::endl;
          unsigned char costx = costmap2D_->getCost(mx,my);
          if(costx > cost){
            cost=costx;
          }
          //std::cout <<"cost = "<< std::hex << (unsigned int)cost << std::endl;
      }
  }
  //unsigned char * cost_max = std::max_element(std::begin(cost), std::end(cost));
  //std::cout <<"cost_max = "<< std::hex << (unsigned int)*cost_max << std::endl;

  /* 前方の時
  x,y,z= -0.21,-0.13,0
  x,y,z= -0.21,0.13,0
  x,y,z= 0.06,0.13,0
  x,y,z= 0.06,-0.13,0
  */
  /* 90度左向いた時
  x,y,z = -0.21,-0.13,0
  x,y,z = -0.21,0.13,0
  x,y,z = 0.06,0.13,0
  x,y,z = 0.06,-0.13,0
  */

  /*
  costmap2D_->worldToMap(x,y,mx,my);
  unsigned char cost2 = costmap2D_->getCost(mx,my);
  //unsigned int j = costmap2D_->getIndex(mx,my);
  std::cout <<"cost = "<< std::hex << (unsigned int)cost2 << std::endl;
  //std::cout <<"j = "<< j << std::endl;

 	double wx,wy;
  costmap2D_->mapToWorld(0,0,wx,wy);
  std::cout <<"wx,wy = "<< wx <<"," << wy << std::endl;
  */
 return cost;

}


/*----------------------------------------------
move()
 called from RobotDrive::navi_move()
    x,y: 絶対番地への移動(基準座標)
    r_yaw: 基準座標での角度。 [rad]

  https://answers.ros.org/question/371925/euler-to-quaternion-c/
------------------------------------------------*/
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

  // timerCallback() 処理に任せる。
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

    last_x_ = target_pose_.pose.position.x;
    last_y_ = target_pose_.pose.position.y;
    //target_pose_.pose.orientation.w;


    nav_state_ = NavState::MOVING;
    moving_cnt_=0;
    moving_err_cnt_=0;

    local_planner_.setPlan(last_global_plan_);
    geometry_msgs::Twist cmd_vel;
    if (local_planner_.isGoalReached()){
      //ROS_INFO("reach");
      std::cout << "reach" << std::endl;
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
      moving_proc();
    }
  }
}

/*----------------------------------------
* moving_proc()
-----------------------------------------*/
void RobotNavi::moving_proc(){
  geometry_msgs::Twist cmd_vel;

  double x,y;
  moving_cnt_++;

  if(moving_cnt_> 70){
    moving_cnt_=0;

    if(get_tf()==true){

      x = target_pose_.pose.position.x;
      y = target_pose_.pose.position.y;

      float off_x = x -  last_x_;
      float off_y = y -  last_y_;

      float cur_dist = std::sqrt(off_x*off_x+off_y*off_y);

      last_x_=x;
      last_y_=y;

      cur_dist = round_my<float>(cur_dist,3);
      //if(cur_dist <= 0.005){
      if(cur_dist <= 0.01){
        moving_err_cnt_++;
        std::cout << "not moving (" << moving_err_cnt_ << ")" << std::endl;
      }
      else{
        moving_err_cnt_=0;
        std::cout << "moving" << std::endl;

      }

      if(moving_err_cnt_ >= 8){
        std::cout << "not moving exceed limits" << std::endl;
        nav_state_ = NavState::STANDBY;
        return;
      }

    }

  }
  //std::cout << "moving" << std::endl;

  // ここの判定がシビアすぎか?
  if (local_planner_.isGoalReached()){
    //ROS_INFO("reach");
    std::cout << "reach!!" << std::endl;
    twist_pub_.publish(cmd_vel);
    nav_state_ = NavState::STANDBY;
  }
  else{
    local_planner_.computeVelocityCommands(cmd_vel);
    twist_pub_.publish(cmd_vel);
  }
}

/*----------------------------------------
* timerCallback()
*
-----------------------------------------*/
void RobotNavi::timerCallback(const ros::TimerEvent& e){
  if (nav_state_ == NavState::WAIT_PLAN){
    ROS_INFO("PLAN");

    // start MakePlan()
    #ifdef USE_ORG_H
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
    #else

      if(MakePlan()!=true){
        nav_state_ = NavState::STANDBY;
        return;       
      }
      last_x_ = target_pose_.pose.position.x;
      last_y_ = target_pose_.pose.position.y;
    #endif

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
    moving_cnt_=0;
    moving_err_cnt_=0;

  }
  // 移動中です。
  else if(nav_state_ == NavState::MOVING){
    //ROS_INFO_THROTTLE(2.0, "MOVING");
    moving_proc();
  }
}

/*----------------------------------------
* get_tf()
*
-----------------------------------------*/
bool RobotNavi::get_tf(){
  std::cout << "get_tf()" << std::endl;

  geometry_msgs::PoseStamped source_pose;
  //source_pose.header.frame_id="dtw_robot1/base_link";
  source_pose.header.frame_id="base_footprint";
  source_pose.header.stamp=ros::Time(0);
  source_pose.pose.orientation.w=1.0;

  //geometry_msgs::PoseStamped target_pose;
  //std::string target_frame="dtw_robot1/map";
  std::string target_frame="map";
  try{
    tf_.waitForTransform(source_pose.header.frame_id, target_frame, ros::Time(0), ros::Duration(1.0));
    // 現在位置を得る  -> target_pose_
    tf_.transformPose(target_frame, source_pose, target_pose_);
    //ROS_INFO("x:%+5.2f, y:%+5.2f,z:%+5.2f",target_pose_.pose.position.x,target_pose_.pose.position.y,target_pose_.pose.position.z);
  }
  catch(...){
    ROS_INFO("tf error");
    return false;
  }

  return true;

}

/*----------------------------------------
* MakePlan()
*
-----------------------------------------*/
bool RobotNavi::MakePlan(){
  std::cout << "MakePlan()" << std::endl;

  if(get_tf()!=true){
    return false;
  }

  geometry_msgs::PoseStamped start = target_pose_;   // 現在位置 / start position

  //std::cout << "last_pose_.header.frame_id=" << last_pose_.header.frame_id << std::endl;
  //std::cout <<  "last_pose_.header.stamp=" << last_pose_.header.stamp << std::endl;
  std::cout << "last_pose_.pose.position.x,y,z=" << last_pose_.pose.position.x <<","<< last_pose_.pose.position.y <<","<< last_pose_.pose.position.z << std::endl;
  std::cout << "last_pose_.pose.orientation.x,y,z,w=" << last_pose_.pose.orientation.x <<","<< last_pose_.pose.orientation.y 
      <<","<< last_pose_.pose.orientation.z <<","<< last_pose_.pose.orientation.w << std::endl;

  if (!global_planner_.makePlan(start, last_pose_, last_global_plan_)){
    ROS_WARN("global plan fail");
    return false;
  }
  return true;
}
//int main(int argc, char** argv){
//  ros::init(argc, argv, "robot_navigation");
//  
//  RobotNavi robot_navi;
//  ros::spin();
//  return 0;
//}
