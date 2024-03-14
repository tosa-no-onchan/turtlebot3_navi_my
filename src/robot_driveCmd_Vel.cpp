/*
* ROS2
* robot_drive.cpp
*
* https://github.com/ros2/turtlebot2_demo/blob/master/turtlebot2_follower/src/follower.cpp
* https://github.com/ros2/turtlebot2_demo/blob/master/turtlebot2_drivers/src/dumb_teleop.cpp
* https://zenn.dev/uchidaryo/articles/ros2-programming-6
*/

#include "turtlebot3_navi_my/robot_driveCmd_Vel.hpp"

#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

using namespace std::chrono_literals;

//! ROS node initialization
void RobotDriveCmd_Vel::init(std::shared_ptr<rclcpp::Node> node,GetTF *getTF,bool navi_use)
{
    node_=node;
    getTF_=getTF;       // add by nishi 2024.2.27

    rclcpp::WallRate rate0(1);
    heartBeat_.init(node_);     // add by nishi 2023.3.8
    rate0.sleep();

    //getTF_.init(node);
    getTF_->init(node);     // changed by nishi 2024.2.27

    if (navi_use==true){
    //    navi_.init(nh,2);
    }

    //set up the publisher for the cmd_vel topic
    //_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    _pub = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);


    //printf("%s",_pub);
    //std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist_<std::allocator<void> >

    _vel_msg.angular.x = _vel_msg.angular.y = _vel_msg.angular.z =0.0;
    _vel_msg.linear.x = _vel_msg.linear.y = _vel_msg.linear.z = 0.0;

    _course_correct=false;
    _after_correct_wait=false;
    _go_curve=false;
    _dumper=false;

    int i=3;
    //ros::Rate rate(50.0); // [Hz]
    rclcpp::WallRate rate(50.0);

    while(i >= 0){
        //getTF_.get();
        getTF_->get();
        rate.sleep();
        //_pub.publish(_vel_msg);
        _pub->publish(_vel_msg);
        i-=1;
    }
}

/*
move()
自分からの相対位置へ移動
    float dist: 自分からの距離
    float d_yaw: 基準座標での角度。 [degree] ロボット座標上の角度では無い
    bool func_f: false[deafult] d_yaw -> 基準座標での角度(今までの処理)
                 true           d_yaw(+/-) -> ロボットからの角度
*/
void RobotDriveCmd_Vel::move(float dist,float d_yaw,bool func_f){
    std::cout << "C move() func_f=" << func_f;
    get_tf(2);
    float r_yaw,d_yawx;
    //tf::Vector3 start_origin = base_tf.getOrigin();
    tf2::Vector3 start_origin = base_tf.getOrigin();

    float start_x = start_origin.getX();
    float start_y = start_origin.getY();

    // 基準座標での角度
    if(func_f==false){
        r_yaw = d_yaw/RADIANS_F;
        d_yawx = d_yaw;
    }
    else{
        r_yaw = _rz + d_yaw/RADIANS_F;
        d_yawx = r_yaw*RADIANS_F;
    }
    std::cout << " d_yawx:"<< d_yawx << std::endl;

    // 目的地を計算
    //float y = self.base_tf.transform.translation.y + dist * math.sin(r_yaw);
    float y = start_y + dist * std::sin(r_yaw);
    //float x = self.base_tf.transform.translation.x + dist * math.cos(r_yaw);
    float x = start_x + dist * std::cos(r_yaw);
    rotate_abs(d_yawx);
    if (dist != 0.0)
        go_abs(x,y);
}
/*
move_abs()
    x,y: 絶対番地への移動(基準座標)
    d_yaw: 基準座標での角度。 [degree]
*/
void RobotDriveCmd_Vel::move_abs(float x,float y,float d_yaw){
    rotate_abs(d_yaw);
    go_abs(x,y);
}

/*
comp_dad() : compute distanse and direction
目的地までの距離と方角を計算する。
    float x:
    float y:
    float &dist:
    float &r_yaw:
    float &r_yaw_off:
*/
void RobotDriveCmd_Vel::comp_dad(float x,float y,float &dist, float &r_yaw, float &r_yaw_off){
    get_tf(1);

    dist=0.0;
    r_yaw=0.0;
    r_yaw_off=0.0;

    //tf::Vector3 cur_origin = base_tf.getOrigin();
    tf2::Vector3 cur_origin = base_tf.getOrigin();

    float cur_x = cur_origin.getX();
    float cur_y = cur_origin.getY();
    // ロボットの向き -> _rz

    float off_x = x - cur_x;
    float off_y = y - cur_y;

    float cur_dist = std::sqrt(off_x*off_x+off_y*off_y);
    cur_dist = round_my<float>(cur_dist,3);

    if(cur_dist > 0.0){
        // ロボットからのターゲットの向き、
        float r_theta = std::atan2(off_y,off_x);   //  [radian] ノーマライズが必要か?

        // 180度表現に変換
        if (abs(r_theta) > 180.0/RADIANS_F){
            if (r_theta > 0.0)
                r_theta -= 360.0/RADIANS_F;
            else
                r_theta +=  360.0/RADIANS_F;
        }

        // ロボットからのターゲットの向き、から、ロボット自体の向きを引く
        float r_theta_off = r_theta - _rz;
        // 180度表現に変換
        if (abs(r_theta_off) > 180.0/RADIANS_F){
            if (r_theta_off > 0.0)
                r_theta_off -= 360.0/RADIANS_F;
            else
            r_theta_off +=  360.0/RADIANS_F;
        }

        //if (r_theta_off > 180.0/RADIANS_F)
        //    r_theta_off = -(360.0/RADIANS_F - r_theta_off);
        //else if (r_theta_off < -180.0/RADIANS_F)
        //    r_theta_off += 360.0/RADIANS_F;

        dist=cur_dist;
        //r_yaw= r_theta+r_theta_robo;
        r_yaw= r_theta;     
        r_yaw_off = r_theta_off;
    }
}

/*
* bool get_tf(int func)
*/
bool RobotDriveCmd_Vel::get_tf(int func){
    bool rc;
    //getTF_.get(func);
    rc=getTF_->get(func);  // changed by nishi 2024.2.27

    //base_tf=getTF_.base_tf;
    base_tf=getTF_->base_tf;    // changed by nishi 2024.2.27

    if (func==2){
        //_rx=getTF_._rx;
        _rx=getTF_->_rx;    // changed by nishi 2024.2.27
        //_ry=getTF_._ry;
        _ry=getTF_->_ry;    // changed by nishi 2024.2.27
        //_rz=getTF_._rz;
        _rz=getTF_->_rz;    // changed by nishi 2024.2.27
        if(log_level>=3)
            std::cout << "_rx: " << _rx << ", _ry: " << _ry << ", _rz: " << _rz << std::endl;
    }
    return rc;
}


/*
go_abs(x,y,isForward=True,speed=0.05)
直進する。
*/
void RobotDriveCmd_Vel::go_abs(float x,float y,float speed ,bool isForward){

    std::cout << "C go_abs()";

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

    //ros::Rate rate(30.0);   // 30[Hz]
    rclcpp::WallRate rate(30.0);

    get_tf();
    //tf::Vector3 start_origin = base_tf.getOrigin();
    tf2::Vector3 start_origin = base_tf.getOrigin();

    //float x = translation.getX();

    float start_x = start_origin.getX();
    float start_y = start_origin.getY();

    float off_x = x - start_x;
    float off_y = y - start_y;
    float off_z = 0.0;

    double start_distance = std::sqrt(off_x*off_x+off_y*off_y+off_z*off_z);
    float current_distance=0.0;

    std::cout << " start_distance=" << round_my<double>(start_distance,3) << std::endl;

    int i = 0;
    int j = 0;
    bool ex_f=false;

    unsigned char cost;

    //Loop to move the turtle in an specified distance
    while(current_distance < start_distance){
        //std::cout << "pub cmd_vel" << std::endl;
        //Publish the velocity
        //_pub.publish(_vel_msg);
        _pub->publish(_vel_msg);

        rate.sleep();

        get_tf(0);

        //tf::Vector3 cur_origin = base_tf.getOrigin();
        tf2::Vector3 cur_origin = base_tf.getOrigin();

        //std::cout << "_x, _y,_z =" << cur_translation.getX() <<", "
        //  << cur_translation.getY() << " , " << cur_translation.getZ() << std::endl;

        float cur_x = cur_origin.getX();
        float cur_y = cur_origin.getY();

        off_x = cur_x - start_x;
        off_y = cur_y - start_y;
        off_z=0.0;

        current_distance = std::sqrt(off_x*off_x+off_y*off_y+off_z*off_z);

        // dumper ON
        if(_dumper==true){
            cost=0;
            //cost=navi_.check_cost(cur_x,cur_y);
            if(cost>0){
                std::cout << "cost=" << (unsigned int)cost << std::endl;
                _vel_msg.linear.x = 0.0;
                // Force the robot to stop
                //_pub.publish(_vel_msg);
                _pub->publish(_vel_msg);
                //while(1){
                //    rate.sleep();
                //}
                return;
            }
        }

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
            j++;
            if(j > 3){
                std::cout << " _dz=" << round_my<double>(_rz*RADIANS_F,3) <<" theta_d=" << round_my<float>(theta_r*RADIANS_F,3)
                    << " theta_ad=" << round_my<float>(theta_ar*RADIANS_F,3) << std::endl;
            }

            if (ex_f == true)
                std::exit(0);

            if (_course_correct == true){
                // 10 [cm] 以上距離がある 時に方向を補正
                if ((start_distance - current_distance) > 0.1 && current_distance > 0.1){
                    // 到達点まで 1[M] 以上　かつ　ズレが 2.0[dgree] か ズレが 4.0[dgree]
                    if ((abs(theta_ar * RADIANS_F) > 4.0) || ((start_distance - current_distance) > 1.0 &&  abs(theta_ar * RADIANS_F) > 2.0)){
                        rotate_off(theta_ar*RADIANS_F,3.0,_go_curve);
                        _vel_msg.linear.x = i_spped;
                        //std::exit(0);
                        //ex_f=true;
                        i=i;
                        if (_after_correct_wait == true){

                            std::cout << " after_correct_wait" << std::endl;
                            get_tf(2);
                            std::cout << " dx,dy,dz=" << round_my<double>(_rx,3) <<"," << round_my<double>(_ry,3) << ","
                                << round_my<double>(_rz,3) << std::endl;
                            while(1)
                                rate.sleep();
                        }
                    }
                }
            }
            if(j > 3){
                std::cout << " current_distance=" << round_my<float>(current_distance,3) << std::endl;
                j=0;
            }
            i=0;
        }    
        //ros::spinOnce();
        rclcpp::spin_some(node_);
    }
    std::cout << "stop" << std::endl;
    // After the loop, stops the robot
    _vel_msg.linear.x = 0.0;
    // Force the robot to stop
    //_pub.publish(_vel_msg);
    _pub->publish(_vel_msg);
}

/*
rotate_abs()
    stop_dz(d_theta) : [deg] 基本座標上の角度
    speed :  5.0  [deg/s]
*/
/*
rotate_abs()
    stop_dz(d_theta) : [deg] 基本座標上の角度
    rad_f : false -> deg / true -> radian
    speed :  5.0  [deg/s]
*/
void RobotDriveCmd_Vel::rotate_abs(float stop_dz,bool rad_f, float speed){
//void RobotDriveCmd_Vel::rotate_abs(float stop_dz,float speed){
    // 目的の角度と速度を設定
    // stop_dz = 180.0 # [deg]
    // speed = 10.0 # [deg/s]

    int turn_plus;
    //geometry_msgs::Twist _vel_msg;

    std::cout << "C rotate_abs()" << std::endl;

    _vel_msg.angular.x = _vel_msg.angular.y = _vel_msg.angular.z =0.0;
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
    //ros::Rate rate(40);   // 40 [Hz]
    rclcpp::WallRate rate(40);
    float rz_min = 10.0;
    bool ok_f = false;
    int lp_cnt=0;
    while(1){
        //_pub.publish(_vel_msg);
        _pub->publish(_vel_msg);

        get_tf(2);
        lp_cnt++;
        if(lp_cnt > 80){
            std::cout << "stop_rz=" << round_my<float>(stop_rz,3) << std::endl;
            lp_cnt=0;
        }

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
        //ros::spinOnce();
        rclcpp::spin_some(node_);
    }
    std::cout << "stop" << std::endl;
    _vel_msg.angular.z = 0.0;   // [rad]
    //Force the robot to stop
    //_pub.publish(_vel_msg);
    _pub->publish(_vel_msg);
    rate.sleep();

    get_tf(2);
}

/* 
void rotate_off()
    d_theta : [deg] ロボット座標上の角度
    speed :  5.0  [deg/s]
*/
void RobotDriveCmd_Vel::rotate_off(float d_theta, float speed, bool go_curve){
    // 目的の角度と速度を設定
    //d_theta = 180.0 # [deg]
    //speed = 10.0 # [deg/s]

    int turn_plus=0;

    std::cout << "C rotate_off() d_theta:" << d_theta << std::endl;

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

    std::cout << " r_theta= " << round_my<float>(r_theta,3) << std::endl;

    // Twist 型のデータ
    //t = Twist()
    //t.linear.x = 0
    // go on straight and turn to target point
    if (go_curve == false){
        _vel_msg.linear.x = 0.0;
    }

    float stop_rz= _rz + r_theta;

    // 180度表現に変換
    if (abs(stop_rz) > 180.0/RADIANS_F){
        if (stop_rz > 0.0)
            stop_rz -= 360.0/RADIANS_F;
        else
            stop_rz +=  360.0/RADIANS_F;
    }

    // stop_rz を目指す
    //ros::Rate rate(40);   // 40 [Hz]
    rclcpp::WallRate rate(40);
    float rz_min = 10.0;
    bool ok_f = false;

    int lp_cnt=0;
    int lp_cnt_chk=0;

    float prev_rz=0.0;

    while(1){
        //_pub.publish(_vel_msg);
        _pub->publish(_vel_msg);
        get_tf(2);
        lp_cnt++;
        if(lp_cnt > 80){
            std::cout << " stop_rz=" << round_my<float>(stop_rz,4) << " _rz="<< round_my<float>(_rz,4) << std::endl;
            lp_cnt=0;
            //小数点以下 2桁は有効
            float _rz_x = round_my<float>(_rz,2);
            // ロボットが動いていない? add by nishi 2024.3.14
            if(prev_rz == _rz_x){
                lp_cnt_chk++;
                if(lp_cnt_chk >= 3){
                    std::cout << " time out" << std::endl;
                    break;
                }
            }
            else{
                lp_cnt_chk=0;
            }
            prev_rz = _rz_x;
        }
        // 最も近い角度で終了します
        if (ok_f == true){
            std::cout << " ok nearly" << std::endl;
            break;
        }
        // 目的の角度です。
        if (abs(stop_rz - _rz) <= rz_dlt){
            //std::cout << "ok just" << std::endl;
            //break;
            // 目的の角度です。
            if (round_my<float>(stop_rz,3) == round_my<float>(_rz,3)){
                std::cout << " ok just" << std::endl;
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
        //ros::spinOnce();
        rclcpp::spin_some(node_);
    }

    std::cout << "stop" << std::endl;
    _vel_msg.angular.z = 0.0; // [rad]
    // Force the robot to stop
    //_pub.publish(_vel_msg);
    _pub->publish(_vel_msg);
    rate.sleep();

    get_tf(2);
}

//! Drive forward a specified distance based on odometry information
bool RobotDriveCmd_Vel::driveForwardOdom(double distance)
{

    //we will record transforms here
    //tf::StampedTransform start_transform;
    tf2::Stamped<tf2::Transform> start_transform;
    //tf::StampedTransform current_transform;

    get_tf(2);
    start_transform = base_tf;

    //we will be sending commands of type "twist"
    //geometry_msgs::Twist base_cmd;
    geometry_msgs::msg::Twist base_cmd;
    //the command will be to go forward at 0.25 m/s
    base_cmd.linear.y = base_cmd.angular.z = 0;
    base_cmd.linear.x = 0.25;

    //ros::Rate rate(10.0);
    rclcpp::WallRate rate(10.0);
    bool done = false;
    while (!done && rclcpp::ok())
    {
        //send the drive command
        //_pub.publish(base_cmd);
        _pub->publish(base_cmd);
        rate.sleep();
        get_tf(2);

        //see how far we've traveled
        //tf::Transform relative_transform = start_transform.inverse() * base_tf;
        tf2::Transform relative_transform = start_transform.inverse() * base_tf;

        double dist_moved = relative_transform.getOrigin().length();

        if(dist_moved > distance) done = true;
    }
    if (done) return true;
    return false;
}

bool RobotDriveCmd_Vel::turnOdom(bool clockwise, double radians)
{
    while(radians < 0) radians += 2*M_PI;
    while(radians > 2*M_PI) radians -= 2*M_PI;

    //we will record transforms here
    //tf::StampedTransform start_transform;
    tf2::Stamped<tf2::Transform> start_transform;

    //tf::StampedTransform current_transform;

    //record the starting transform from the odometry to the base frame
    get_tf(2);
    start_transform = base_tf;

    //we will be sending commands of type "twist"
    //geometry_msgs::Twist base_cmd;
    geometry_msgs::msg::Twist base_cmd;
    //the command will be to turn at 0.75 rad/s -> 42.97 degrees/s
    base_cmd.linear.x = 0.0;
    base_cmd.linear.y = 0.0;
    //base_cmd.angular.z = 0.75;
    base_cmd.angular.z = 5.0/RADIANS_F;

    if (clockwise) 
        base_cmd.angular.z = -base_cmd.angular.z;

    //the axis we want to be rotating by
    //tf::Vector3 desired_turn_axis(0,0,1);
    tf2::Vector3 desired_turn_axis(0,0,1);

    if (!clockwise) 
        desired_turn_axis = -desired_turn_axis;

    //ros::Rate rate(10.0);
    rclcpp::WallRate rate(10.0);


    bool done = false;
    while (!done && rclcpp::ok())
    {
        //send the drive command
        //_pub.publish(base_cmd);
        _pub->publish(base_cmd);
        rate.sleep();

        get_tf(2);

        //tf::Transform relative_transform = start_transform.inverse() * base_tf;
        tf2::Transform relative_transform = start_transform.inverse() * base_tf;

        //tf::Vector3 actual_turn_axis = relative_transform.getRotation().getAxis();
        tf2::Vector3 actual_turn_axis = relative_transform.getRotation().getAxis();
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

/*-----------------------
- robot_navi call routine
--------------------------*/
/*
navi_move()
    x,y: 絶対番地への移動(基準座標)
    r_yaw: 基本座標上の 角度。 [rad]
    r_yaw_off: ロボットの向きからの 角度。 [rad]
*/
bool RobotDriveCmd_Vel::navi_move(float x,float y,float r_yaw,float r_yaw_off){
    if(r_yaw_off != 0.0){
        rotate_off(r_yaw_off*RADIANS_F);
    }
    //navi_.move(x,y,r_yaw);
    return true;
}

/*
navi_map_save()
*/
void RobotDriveCmd_Vel::navi_map_save(){
    //navi_.map_save();
}




