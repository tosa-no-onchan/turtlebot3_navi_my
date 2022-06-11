/*
robot_drive.cpp
https://wiki.ros.org/pr2_controllers/Tutorials/Using%20the%20base%20controller%20with%20odometry%20and%20transform%20information

https://www.k-cube.co.jp/wakaba/server/func/math_h.html

https://answers.ros.org/question/50113/transform-quaternion/

build
$ catkin_make --pkg turtlebot3_navi_my

$ rosrun turtlebot3_navi_my drive_base
*/

#include "turtlebot3_navi_my/robot_drive.h"

//! ROS node initialization
void RobotDrive::init(ros::NodeHandle &nh)
{
  nh_ = nh;
  //set up the publisher for the cmd_vel topic
  _pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);


  _vel_msg.angular.x = _vel_msg.angular.y = _vel_msg.angular.x =0.0;
  _vel_msg.linear.x = _vel_msg.linear.y = _vel_msg.linear.z = 0.0;

  _course_correct=false;
  _after_correct_wait=false;

  //wait for the listener to get the first message
  //listener_.waitForTransform("base_footprint","odom", ros::Time(0), ros::Duration(1.0));
  //listener_.waitForTransform("base_footprint","map", ros::Time(0), ros::Duration(1.0));
  listener_.waitForTransform("map","base_footprint", ros::Time(0), ros::Duration(1.0));
  sleep(1);

  int i=3;
  ros::Rate rate(50.0); // [Hz]

  while(i >= 0){
      get_tf();
      rate.sleep();
      _pub.publish(_vel_msg);
      i-=1;
  }

}

/*
move()
自分からの相対位置へ移動
    dist: 自分からの距離
    d_yaw: 基準座標での角度。 [degree] ロボット座標上の角度では無い
*/
void RobotDrive::move(float dist,float d_yaw){
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
void RobotDrive::move_abs(float x,float y,float d_yaw){
    rotate_abs(d_yaw);
    go_abs(x,y);
}

/*
* void get_tf(int func)
*/
void RobotDrive::get_tf(int func){
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
    std::cout << "_rx: " << _rx << ", _ry: " << _ry << ", _rz: " << _rz << std::endl;
  }
}


/*
go_abs(x,y,isForward=True,speed=0.05)
直進する。
*/
void RobotDrive::go_abs(float x,float y,float speed ,bool isForward){

    std::cout << "go_abs" << std::endl;

    float i_spped;

    if(isForward)
        i_spped = abs(speed);
    else
        i_spped = -abs(speed);

    _vel_msg.linear.x = i_spped;
    _vel_msg.linear.y = 0.0;
    _vel_msg.linear.z = 0.0;
    _vel_msg.angular.x = 0.0;
    _vel_msg.angular.y = 0.0;
    _vel_msg.angular.z = 0.0;

    ros::Rate rate(30.0);   // 30[Hz]

    get_tf();
    tf::Vector3 start_origin = base_tf.getOrigin();

    //float x = translation.getX();

    float start_x = start_origin.getX();
    float start_y = start_origin.getY();

    float off_x = x - start_x;
    float off_y = y - start_y;
    float off_z = 0.0;

    double start_distance = std::sqrt(off_x*off_x+off_y*off_y+off_z*off_z);
    float current_distance=0.0;

    std::cout << "start_distance=" << round_my<double>(start_distance,3) << std::endl;

    int i = 0;
    bool ex_f=false;

    //Loop to move the turtle in an specified distance
    while(current_distance < start_distance){
      //std::cout << "pub cmd_vel" << std::endl;
      //Publish the velocity
      _pub.publish(_vel_msg);

      rate.sleep();

      get_tf(0);

      tf::Vector3 cur_origin = base_tf.getOrigin();

      //std::cout << "_x, _y,_z =" << cur_translation.getX() <<", "
      //  << cur_translation.getY() << " , " << cur_translation.getZ() << std::endl;

      float cur_x = cur_origin.getX();
      float cur_y = cur_origin.getY();

      off_x = cur_x - start_x;
      off_y = cur_y - start_y;
      off_z=0.0;

      current_distance = std::sqrt(off_x*off_x+off_y*off_y+off_z*off_z);

      i+=1;
      if (i > 5){
        // 自分からの目的地の方角
        float off_target_x = x - cur_x;
        float off_target_y = y - cur_y;

        float theta_r = std::atan2(off_target_y,off_target_x);   //  [ragian]

        // 自分の方向を減算
        float theta_ar = theta_r - _rz;

        // 後ろ向き
        if (abs(theta_ar) > 180.0/RADIANS_F){
            if (theta_ar > 0.0)
                theta_ar -= 360.0/RADIANS_F;
            else
                theta_ar += 360.0/RADIANS_F;
        }
        std::cout << "_dz=" << round_my<double>(_rz*RADIANS_F,3) <<" theta_d=" << round_my<float>(theta_r*RADIANS_F,3)
          << " theta_ad=" << round_my<float>(theta_ar*RADIANS_F,3) << std::endl;

        if (ex_f == true)
            std::exit(0);

        if (_course_correct == true){
            // 10 [cm] 以上距離がある 時に方向を補正
            if ((start_distance - current_distance) > 0.1 && current_distance > 0.1){
                if (abs(theta_ar * RADIANS_F) > 5.0){
                    rotate_off(theta_ar*RADIANS_F,3.0);
                    _vel_msg.linear.x = i_spped;
                    //std::exit(0);
                    //ex_f=true;
                    i=i;
                    if (_after_correct_wait == true){

                        std::cout << "after_correct_wait" << std::endl;
                        get_tf(2);
                        std::cout << "dx,dy,dz=" << round_my<double>(_rx,3) <<"," << round_my<double>(_ry,3) << ","
                          << round_my<double>(_rz,3) << std::endl;
                        while(1)
                            rate.sleep();
                    }
                }
            }
        }
        std::cout << "current_distance=" << round_my<float>(current_distance,3) << std::endl;
        i=0;
      }    
    }
    std::cout << "stop" << std::endl;
    // After the loop, stops the robot
    _vel_msg.linear.x = 0.0;
    // Force the robot to stop
    _pub.publish(_vel_msg);
}

/*
rotate_abs()
    stop_dz(d_theta) : [deg] 基本座標上の角度
    speed :  5.0  [deg/s]
*/
void RobotDrive::rotate_abs(float stop_dz,float speed){
    // 目的の角度と速度を設定
    // stop_dz = 180.0 # [deg]
    // speed = 10.0 # [deg/s]

    int turn_plus;
    //geometry_msgs::Twist _vel_msg;

    _vel_msg.angular.x = _vel_msg.angular.y = _vel_msg.angular.x =0.0;
    _vel_msg.linear.x = _vel_msg.linear.y = _vel_msg.linear.z = 0.0;

    std::cout << "start rotate_abs stop_dz=" << stop_dz << std::endl;

    //# turn plus
    if (stop_dz >= 0.0){
        turn_plus = 1;   // reverse clock(left) rotate
        //#self.vel_msg.angular.z = speed * 3.1415 / 180.0 # [rad]
        //#self.vel_msg.angular.z = speed # [rad]
        _vel_msg.angular.z = speed / RADIANS_F;  // [rad]
    }
    // turn minus
    else{
        turn_plus = -1;   // regura clock(right) rotate
        //#self.vel_msg.angular.z = speed * 3.1415 / -180.0 # [rad]
        _vel_msg.angular.z = speed / RADIANS_F * -1.0;  //  [rad]
    }

    std::cout << "turn_plus=" << turn_plus << std::endl;

    float rz_dlt = abs(speed / RADIANS_F) * 0.25;
    // #z_wind = abs(speed / RADIANS_F) * 2.0;
    float rz_wind = rz_dlt * 3.0;

    //# Twist 型のデータ
    //#t = Twist()
    //#t.linear.x = 0
    _vel_msg.linear.x = 0.0;

    float stop_rz = stop_dz/ RADIANS_F;
    // 180度表現に変換
    if (abs(stop_rz) > 180.0/RADIANS_F){
        if (stop_rz > 0.0)
            stop_rz -= 360.0/RADIANS_F;
        else
            stop_rz +=  360.0/RADIANS_F;
    }

    get_tf(2);
    std::cout << "_rx,_ry,_rz=" << _rx << ","<< _ry << "," << _rz << std::endl;

    float rz_off = stop_rz - _rz;

    if (abs(rz_off) <= rz_dlt)
        return;

    std::cout << "dz_off=" << rz_off * RADIANS_F << std::endl;

    // 逆回りが近い
    if (abs(rz_off) > 180.0/RADIANS_F){
        if (rz_off > 0.0)
            rz_off -= 360.0/RADIANS_F;
        else
            rz_off += 360.0/RADIANS_F;
    }
    if (abs(rz_off) <= 10.0/RADIANS_F){      //# 角度差が 10.0 度以内 であれば、補正回転にする
        std::cout << "adjust angle" << std::endl;
        // 左回り
        if (rz_off >= 0.0){
            _vel_msg.angular.z = abs(_vel_msg.angular.z);
            turn_plus = 1;
        }
        // 右回り
        else{
            _vel_msg.angular.z = abs(_vel_msg.angular.z) * -1.0;
            turn_plus = -1;
        }
    }
    else{
        std::cout << ">non adjust angle" << std::endl;
    }

    std::cout << "turn_plus=" << turn_plus << std::endl;

    //print 'test1'
    //sys.exit()

    // stop_rz を目指す
    // rate = rospy.Rate(30)   # 30 [Hz]
    ros::Rate rate(40);   // 40 [Hz]
    float rz_min = 10.0;
    bool ok_f = false;
    while(1){
        _pub.publish(_vel_msg);

        get_tf(2);
        std::cout << "stop_rz=" << round_my<float>(stop_rz,3) << std::endl;

        // 最も近い角度で終了します
        if (ok_f == true){
            std::cout << "ok nearly" << std::endl;
            break;
        }
        // 目的の角度です。
        if (abs(_rz - stop_rz) <= rz_dlt){
          //  std::cout << "ok just" << std::endl;
          //  break;
          // 目的の角度です。
          if (round_my<float>(stop_rz,3) == round_my<float>(_rz,3)){
            std::cout << "ok just" << std::endl;
            break;
          }
        }
        //rz_off = abs(abs(stop_rz) - abs(self.rz))
        float rz_off = stop_rz - _rz;
        if (rz_off > 180.0/RADIANS_F){
            rz_off = -(360.0/RADIANS_F - rz_off);
        }
        else if (rz_off < -180.0/RADIANS_F){
            rz_off += 360.0/RADIANS_F;
        }
        rz_off=abs(rz_off);
        if (rz_off <= rz_wind){
            // 最小の角度を求める
            // print 'rz_off=',rz_off 
            if (rz_off <= rz_min){
                rz_min= rz_off;
            }
            // 最小の角度を通り過ぎ
            else{
                // 1クロック逆戻り
                _vel_msg.angular.z *= -1.0;
                ok_f = true;
            }
        }
        rate.sleep();
    }
    std::cout << "stop" << std::endl;
    _vel_msg.angular.z = 0.0;   // [rad]
    //Force the robot to stop
    _pub.publish(_vel_msg);
    rate.sleep();

    get_tf(2);
}

/* 
void rotate_off()
    d_theta : [deg] ロボット座標上の角度
    speed :  5.0  [deg/s]
*/
void RobotDrive::rotate_off(float d_theta, float speed){
    // 目的の角度と速度を設定
    //d_theta = 180.0 # [deg]
    //speed = 10.0 # [deg/s]

    int turn_plus=0;

    std::cout << "start rotate_off d_theta=" << d_theta << std::endl;

    float rz_dlt = abs(speed/RADIANS_F) * 0.25;
    float rz_wind = rz_dlt * 3.0;

    float r_theta = d_theta/RADIANS_F;
    if (abs(r_theta) <= rz_dlt)
        return;

    if (d_theta >= 0.0){
        turn_plus = 1;   // reverse clock(left) rotate
        //_vel_msg.angular.z = speed * 3.1415 / 180.0; //[rad]
        //_vel_msg.angular.z = speed; // [rad]
        _vel_msg.angular.z = speed/RADIANS_F;  // [rad]
    }
    else{
        turn_plus = -1;   // clock(right) rotate
        //_vel_msg.angular.z = speed * 3.1415 / -180.0; // [rad]
        _vel_msg.angular.z = speed/RADIANS_F * -1.0;  // [rad]
    }

    get_tf(2);
    //#print 'rx,ry,rz=',self.rx,self.ry,self.rz

    std::cout << "r_theta= " << round_my<float>(r_theta,3) << std::endl;

    // Twist 型のデータ
    //t = Twist()
    //t.linear.x = 0
    _vel_msg.linear.x = 0.0;

    float stop_rz= _rz + r_theta;

    // 180度表現に変換
    if (abs(stop_rz) > 180.0/RADIANS_F){
        if (stop_rz > 0.0)
            stop_rz -= 360.0/RADIANS_F;
        else
            stop_rz +=  360.0/RADIANS_F;
    }

    // stop_rz を目指す
    ros::Rate rate(40);   // 40 [Hz]
    float rz_min = 10.0;
    bool ok_f = false;

    while(1){
        _pub.publish(_vel_msg);

        get_tf(2);
        std::cout << "stop_rz=" << round_my<float>(stop_rz,4) << std::endl;

        // 最も近い角度で終了します
        if (ok_f == true){
            std::cout << "ok nearly" << std::endl;
            break;
        }
        // 目的の角度です。
        if (abs(stop_rz - _rz) <= rz_dlt){
          //std::cout << "ok just" << std::endl;
          //break;
          // 目的の角度です。
          if (round_my<float>(stop_rz,3) == round_my<float>(_rz,3)){
            std::cout << "ok just" << std::endl;
            break;
          }
        }

        float rz_off = stop_rz - _rz;
        if (rz_off > 180.0/RADIANS_F)
            rz_off = -(360.0/RADIANS_F - rz_off);

        else if (rz_off < -180.0/RADIANS_F)
            rz_off += 360.0/RADIANS_F;

        rz_off=abs(rz_off);
        if (rz_off <= rz_wind){
            //print "rz_off=",rz_off," rz_min=",rz_min
            // 最小の角度を求める
            if (rz_off <= rz_min)
                rz_min = rz_off;

            // 最小の角度を通り過ぎ
            else{
                // 1クロック逆戻り
                _vel_msg.angular.z *= -1.0;
                ok_f = true;
                //break;
            }
        }
        rate.sleep();
    }

    std::cout << "stop" << std::endl;
    _vel_msg.angular.z = 0.0; // [rad]
    // Force the robot to stop
    _pub.publish(_vel_msg);
    rate.sleep();

    get_tf(2);
}

//! Drive forward a specified distance based on odometry information
bool RobotDrive::driveForwardOdom(double distance)
{
  
  //we will record transforms here
  tf::StampedTransform start_transform;
  //tf::StampedTransform current_transform;

  get_tf(2);
  start_transform = base_tf;
  
  //we will be sending commands of type "twist"
  geometry_msgs::Twist base_cmd;
  //the command will be to go forward at 0.25 m/s
  base_cmd.linear.y = base_cmd.angular.z = 0;
  base_cmd.linear.x = 0.25;
  
  ros::Rate rate(10.0);
  bool done = false;
  while (!done && nh_.ok())
  {
    //send the drive command
    _pub.publish(base_cmd);
    rate.sleep();
    get_tf(2);

    //see how far we've traveled
    tf::Transform relative_transform = start_transform.inverse() * base_tf;
    double dist_moved = relative_transform.getOrigin().length();

    if(dist_moved > distance) done = true;
  }
  if (done) return true;
  return false;
}

bool RobotDrive::turnOdom(bool clockwise, double radians)
{
  while(radians < 0) radians += 2*M_PI;
  while(radians > 2*M_PI) radians -= 2*M_PI;
  
  //we will record transforms here
  tf::StampedTransform start_transform;
  //tf::StampedTransform current_transform;

  //record the starting transform from the odometry to the base frame
  get_tf(2);
  start_transform = base_tf;
  
  //we will be sending commands of type "twist"
  geometry_msgs::Twist base_cmd;
  //the command will be to turn at 0.75 rad/s -> 42.97 degrees/s
  base_cmd.linear.x = 0.0;
  base_cmd.linear.y = 0.0;
  //base_cmd.angular.z = 0.75;
  base_cmd.angular.z = 5.0/RADIANS_F;

  if (clockwise) 
    base_cmd.angular.z = -base_cmd.angular.z;
  
  //the axis we want to be rotating by
  tf::Vector3 desired_turn_axis(0,0,1);

  if (!clockwise) 
    desired_turn_axis = -desired_turn_axis;
  
  ros::Rate rate(10.0);
  bool done = false;
  while (!done && nh_.ok())
  {
    //send the drive command
    _pub.publish(base_cmd);
    rate.sleep();

    get_tf(2);

    tf::Transform relative_transform = start_transform.inverse() * base_tf;
    
    tf::Vector3 actual_turn_axis = relative_transform.getRotation().getAxis();
    double angle_turned = relative_transform.getRotation().getAngle();
    //if ( fabs(angle_turned) < 1.0e-2) continue;
    if ( fabs(angle_turned) < 0.01) continue;

    if ( actual_turn_axis.dot( desired_turn_axis ) < 0 ) 
      angle_turned = 2 * M_PI - angle_turned;

    if (angle_turned > radians) 
      done = true;
  }
  if (done) return true;
  return false;
}

