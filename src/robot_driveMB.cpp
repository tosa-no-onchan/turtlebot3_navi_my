/*
robot_driveMB.cpp
robot_drivee for move__base

https://wiki.ros.org/pr2_controllers/Tutorials/Using%20the%20base%20controller%20with%20odometry%20and%20transform%20information

https://www.k-cube.co.jp/wakaba/server/func/math_h.html

https://answers.ros.org/question/50113/transform-quaternion/

build
$ catkin_make --pkg turtlebot3_navi_my

$ rosrun turtlebot3_navi_my drive_base
*/

#include "turtlebot3_navi_my/robot_driveMB.h"

#ifdef TEST_MOVE_NISI_1
void RobotDriveMB::goalCallback(const move_base_msgs::MoveBaseActionResult msg){
    //if data.status.status == 3: # reached
    //    now_xy=self.get_odom()
    //    self.sts=2
    //else:
    //    self.sts=3
    //    rospy.loginfo("statusCB error ! data.status.status=%d", data.status.status) 
    //std::cout << "msg.status.status=" << (int)msg.status.status << std::endl;

    u_int8_t status_id = msg.status.status;

    if(id_==1){
        switch(status_id){
        case 0:
            //ゴールに到達・もしくはゴールに到達して待機中。
            std::cout << "MB PENDING"<< std::endl;
            id_=2;
            break;
        case 1:
            //移動中
            std::cout << "MB ACTIVE"<< std::endl;
            break;       
        case 2:
            std::cout << "MB PREEMPTED"<< std::endl;
            break;
        case 3:
            //ゴールに到達・もしくはゴールに到達して待機中。
            std::cout << "MB SUCCEEDED"<< std::endl;
            //if(rotate_f !=true){
                id_=2;
            //}
            break;
        case 4:
            // エラーの時
            std::cout << "MB ABORTED"<< std::endl;
            id_=4;
            break;
        case 5:
            std::cout << "MB REJECTED"<< std::endl;
            id_=5;
            break;
        case 6:
            std::cout << "MB PREEMPTING"<< std::endl;
            id_=6;
            break;
        case 7:
            std::cout << "MB RECALLING"<< std::endl;
            break;
        case 8:
            std::cout << "MB RECALLED"<< std::endl;
            break;
        case 9:
            std::cout << "MB LOST"<< std::endl;
            id_=9;
            break;
        }
    }    

}
#endif

#ifdef TEST_MOVE_NISI_2
void RobotDriveMB::navStatusCallBack(const actionlib_msgs::GoalStatusArray::ConstPtr &status)
{
    int status_id = -1;
    //uint8 PENDING         = 0  
    //uint8 ACTIVE          = 1 
    //uint8 PREEMPTED       = 2
    //uint8 SUCCEEDED       = 3
    //uint8 ABORTED         = 4
    //uint8 REJECTED        = 5
    //uint8 PREEMPTING      = 6
    //uint8 RECALLING       = 7
    //uint8 RECALLED        = 8
    //uint8 LOST            = 9

    if (!status->status_list.empty()){
        actionlib_msgs::GoalStatus goalStatus = status->status_list[0];
        status_id = goalStatus.status;
    }

    if(id_==1){
        switch(status_id){
        case 0:
            //ゴールに到達・もしくはゴールに到達して待機中。
            std::cout << "MB2 PENDING"<< std::endl;
            //id_=2;
            break;
        case 1:
            //移動中
            std::cout << "MB2 ACTIVE"<< std::endl;
            break;       
        case 2:
            std::cout << "MB2 PREEMPTED"<< std::endl;
            break;
        case 3:
            //ゴールに到達・もしくはゴールに到達して待機中。
            std::cout << "MB2 SUCCEEDED"<< std::endl;
            //if(rotate_f !=true){
                //id_=2;
            //}
            break;
        case 4:
            // エラーの時
            std::cout << "MB2 ABORTED"<< std::endl;
            //id_=4;
            break;
        case 5:
            std::cout << "MB2 REJECTED"<< std::endl;
            //id_=5;
            break;
        case 6:
            std::cout << "MB2 PREEMPTING"<< std::endl;
            //id_=6;
            break;
        case 7:
            std::cout << "MB2 RECALLING"<< std::endl;
            break;
        case 8:
            std::cout << "MB2 RECALLED"<< std::endl;
            break;
        case 9:
            std::cout << "MB2 LOST"<< std::endl;
            //id_=9;
            break;
        }
    }

}
#endif

void RobotDriveMB::init(ros::NodeHandle &nh,bool navi_use){
    nh_=nh;

    pub_ = nh_.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);

    #ifdef TEST_MOVE_NISI_1
        goal_sub_ = nh_.subscribe("move_base/result", 10, &RobotDriveMB::goalCallback, this);
        //move_base/result (move_base_msgs/MoveBaseActionResult) 
    #endif
    #ifdef TEST_MOVE_NISI_2
        //goal_sub_ = nh_.subscribe("move_base/result", 10, &DriveMB::goalCallback, this);
        //ros::Subscriber move_base_status_sub;
        //goal_sub_ = nh_.subscribe<actionlib_msgs::GoalStatusArray>("/move_base/status", 10, &DriveMB::navStatusCallBack);
        goal_sub2_ = nh_.subscribe("/move_base/status", 10, &RobotDriveMB::navStatusCallBack,this);
    #endif

    listener_.waitForTransform("map","base_footprint", ros::Time(0), ros::Duration(1.0));

    sleep(1);
}

void RobotDriveMB::exec_pub(float x,float y,float r_yaw,bool rotate_f){

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, r_yaw);        // q.setRPY( 0, 0, 0 );  // Create this quaternion from roll/pitch/yaw (in radians)
    q=q.normalize();

    //std::cout << "x: " << q.getX() << " y: " << q.getY() << 
    //                " z: " << q.getZ() << " w: " << q.getW() << std::endl;

    ros::Rate one_sec(1);
    one_sec.sleep();
     
    ros::Time time = ros::Time::now();
    geometry_msgs::PoseStamped goal_point;
 
    goal_point.pose.position.x = x;
    goal_point.pose.position.y = y;
    goal_point.pose.position.z =  0.0;
    goal_point.pose.orientation.x = q.getX();
    goal_point.pose.orientation.y = q.getY();
    goal_point.pose.orientation.z = q.getZ();
    goal_point.pose.orientation.w = q.getW();
    goal_point.header.stamp = time;
    goal_point.header.frame_id = "map";
 
    ros::Rate rate(50.0); // [Hz]

    pub_.publish(goal_point);
    id_=1;

    int i=0;
    int moving_cnt =0;
    int moving_err_cnt=0;

    float last_x=0.0;
    float last_y=0.0;

    while(1){
        rate.sleep();
        ros::spinOnce();
        if(id_ >= 2)
            break;

        moving_cnt ++;
        if(rotate_f==false){
            if(moving_cnt > 150){
                get_tf();
                tf::Vector3 cur_origin = base_tf.getOrigin();
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
                    id_=10;
                }

                moving_cnt=0;
            }
        }
        else{
            if((moving_cnt % 150) == 0){
                std::cout << "rotate wait"<< std::endl;
            }
            if(moving_cnt > 1500){
                std::cout << "move_base exceed rotate count"<< std::endl;
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
* void get_tf(int func)
*/
void RobotDriveMB::get_tf(int func){
  ros::Rate rate(70.0);
  while (1)
  {
    try
    {
      //listener_.lookupTransform("base_footprint","odom", ros::Time(0), base_tf);
      //listener_.lookupTransform("base_footprint","map", ros::Time(0), base_tf);
      listener_.lookupTransform("map","base_footprint", ros::Time(0), base_tf);
    }
    catch (tf::TransformException ex)
    {
      //ROS_ERROR("%s",ex.what());
      rate.sleep();
      continue;
      //break;
    }
    break;
  }

  if (func==2){
    //Quaternion getRotation()
    tf::Quaternion q = base_tf.getRotation();

    tf::Matrix3x3 m(q);
    //double roll, pitch, yaw;
    m.getRPY(_rx, _ry, _rz);
    if(log_level>=3)
        std::cout << "_rx: " << _rx << ", _ry: " << _ry << ", _rz: " << _rz << std::endl;
  }
}

/*
move()
自分からの相対位置へ移動
    dist: 自分からの距離
    d_yaw: 基準座標での角度。 [degree] ロボット座標上の角度では無い
*/
void RobotDriveMB::move(float dist,float d_yaw){
    get_tf();
    tf::Vector3 start_origin = base_tf.getOrigin();

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
void RobotDriveMB::move_abs(float x,float y,float d_yaw){

    std::cout << "RobotDriveMB::move_abs() called"<< std::endl;

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
void RobotDriveMB::comp_dad(float x,float y,float &dist, float &r_yaw, float &r_yaw_off){
    get_tf(1);

    dist=0.0;
    r_yaw=0.0;
    r_yaw_off=0.0;

    tf::Vector3 cur_origin = base_tf.getOrigin();

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
void RobotDriveMB::go_abs(float x,float y,float speed ,bool isForward){

    std::cout << "go_abs" << std::endl;

}

/*
rotate_abs()
    stop_dz(d_theta) : [deg] 基本座標上の角度
    rad_f : false -> deg / true -> radian
    speed :  5.0  [deg/s]
*/
void RobotDriveMB::rotate_abs(float stop_dz,bool rad_f,float speed){
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
    tf::Vector3 start_origin = base_tf.getOrigin();

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
void RobotDriveMB::rotate_off(float d_theta, float speed, bool go_curve){
    // 目的の角度と速度を設定
    //d_theta = 180.0 # [deg]
    //speed = 10.0 # [deg/s]
    std::cout << "start rotate_off d_theta=" << d_theta << std::endl;

    float r_theta = d_theta/RADIANS_F;

    get_tf(2);      // _rx, _ry, _rz

    tf::Vector3 start_origin = base_tf.getOrigin();

    float start_x = start_origin.getX();
    float start_y = start_origin.getY();

    //rotate_f=true;

    exec_pub(start_x,start_y,r_theta+_rz,true);

}

//! Drive forward a specified distance based on odometry information
bool RobotDriveMB::driveForwardOdom(double distance)
{
  

  return false;
}

bool RobotDriveMB::turnOdom(bool clockwise, double radians)
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
///void RobotDriveMB::navi_move(float x,float y,float r_yaw){
bool RobotDriveMB::navi_move(float x,float y,float r_yaw,float r_yaw_off){
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
void RobotDriveMB::navi_map_save(){
    //navi_.map_save();
}