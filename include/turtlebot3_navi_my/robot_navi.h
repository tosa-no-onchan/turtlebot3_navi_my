/*
* robot_navi.h
*
*
* http://wiki.ros.org/costmap_2d/
*/

#ifndef ROBOT_NAVI_H
#define ROBOT_NAVI_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/trajectory_planner_ros.h>
#include <navfn/navfn_ros.h>

enum class NavState{
  STANDBY,
  WAIT_PLAN,
  MOVING
};

class RobotNavi {
private:
  int func_;

public:

  ros::NodeHandle nh_;
#if ROS_VERSION_MINIMUM(1,14,0)
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;
#endif
  tf::TransformListener tf_; // obsolute
  ros::Publisher twist_pub_;
  ros::Subscriber goal_sub_;
  ros::Timer timer_;

  NavState nav_state_;
  geometry_msgs::PoseStamped last_pose_;
  std::vector<geometry_msgs::PoseStamped> last_global_plan_;

  costmap_2d::Costmap2DROS global_costmap_;
  costmap_2d::Costmap2DROS local_costmap_;
  navfn::NavfnROS global_planner_;
  base_local_planner::TrajectoryPlannerROS local_planner_;

  // local cost map access add by nishi 2022.7.7
  costmap_2d::Costmap2D* costmap2D_;
  std::vector<geometry_msgs::Point> footprint_spec_;

#if ROS_VERSION_MINIMUM(1,14,0)
  RobotNavi() : tfBuffer(), tfListener(tfBuffer), global_costmap_("global_costmap", tfBuffer), local_costmap_("local_costmap", tfBuffer){}
#else
  // here melocic.
  RobotNavi() : global_costmap_("global_costmap", tf_), local_costmap_("local_costmap", tf_){}
#endif

  void init(ros::NodeHandle &nh,int func=0);
  void goalCallback(const geometry_msgs::PoseStamped msg);
  void timerCallback(const ros::TimerEvent& e);
  /*
  move()
      x,y: 絶対番地への移動(基準座標)
      r_yaw: 基準座標での角度。 [rad]
  */
  void move(float x,float y,float r_yaw);
  bool MakePlan();

  unsigned char check_cost(float x=0.0,float y=0.0);
  void map_save();

};

//int main(int argc, char** argv){
//  ros::init(argc, argv, "move_navigation");
  
//  MoveNavi move_navi;
//  ros::spin();
//  return 0;
//}

#endif      // ROBOT_NAVI_H
