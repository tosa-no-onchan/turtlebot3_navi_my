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

#include <thread>
#include <math.h>

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

void RobotDriveCmd_Vel::th_check_cource_obstacle(float x, float y){
    std::cout << "C th_check_cource_obstacle() "<< std::endl;
    black_cnt_=0;
    th_check_cource_obstacle_f=true;

    rclcpp::WallRate rate(1.0);

    int rc;
    float x1=x;
    float y1=y;

    while(th_check_cource_obstacle_f==true){
        rate.sleep();
        std::cout << "C th_check_cource_obstacle(): wake up "<< std::endl;

        if(get_costmap_->get() != true)
        {
            std::cout << "  getcost_map error occured , then move_abs_auto_select() is not executable!!" << std::endl;
            //return false;
        }
        else if(true==getTF_->get()){
            //base_tf=getTF_.base_tf;
            tf2::Vector3 cur_origin = getTF_->base_tf.getOrigin();

            float cx = cur_origin.getX();
            float cy = cur_origin.getY();

            //std::cout << " cx: " << cx << ", cy: " << cy << std::endl;

            // 自分からの目的地のずれ
            float off_x = x - cx;
            float off_y = y - cy;

            // 距離
            double dist = std::sqrt(off_x*off_x + off_y*off_y);
            // 距離が、 0.5[M] になるまでチェックする。
            if(dist >= 0.5){

                // 目的地の方角
                float theta_r = std::atan2(off_y,off_x);   //  [radian]

                //rc=check_cource_obstacle_comb(*get_map_, *get_costmap_, rx, ry,x, y, 0.3, 0);
                //rc=check_cource_obstacle_comb_ptr(get_map_, get_costmap_, rx, ry,x, y, 0.3, 0);

                // ロボットの前方 0.4 - 1.0[M] 以内のチェックをする。
                x1 = 0.4 * std::cos(theta_r) + cx;
                y1 = 0.4 * std::sin(theta_r) + cy;
                //std::cout << " x1: " << x1 << ", y1: " << y1 << std::endl;
                black_cnt_=get_costmap_->check_cource_obstacle(cx, cy, x1, y1, 0.3, 0);
                std::cout << "  black_cnt:" << black_cnt_ << std::endl;

            }
        }
        else{
            std::cout << "C th_check_cource_obstacle(): getTF_->get() error "<< std::endl;
        }
    }
    std::cout << "C th_check_cource_obstacle(): #99 end "<< std::endl;

}

/*
move()
自分からの相対位置へ移動
    float dist: 自分からの距離
                   > 0 前進
                   < 0 後退
    float d_yaw: ロボットからの角度。 [degree] 
*/
void RobotDriveCmd_Vel::move(float dist,float d_yaw){
    bool isBack=false;
    std::cout << "C move() ";
    get_tf(2);
    float r_yaw,d_yawx;
    //tf::Vector3 start_origin = base_tf.getOrigin();
    tf2::Vector3 start_origin = base_tf.getOrigin();

    float start_x = start_origin.getX();
    float start_y = start_origin.getY();

    // ロボットからの角度
    r_yaw = _rz + d_yaw/RADIANS_F;
    d_yawx = r_yaw*RADIANS_F;

    std::cout << " d_yawx:"<< d_yawx << std::endl;

    if(dist < 0){
        isBack=true;
    }

    std::cout << " start_x:"<< start_x << " start_y:"<< start_y << std::endl;

    // 目的地を計算
    //float y = self.base_tf.transform.translation.y + dist * math.sin(r_yaw);
    float y = start_y + dist * std::sin(r_yaw);
    //float x = self.base_tf.transform.translation.x + dist * math.cos(r_yaw);
    float x = start_x + dist * std::cos(r_yaw);

    std::cout << " x:"<< x << " y:"<< y << std::endl;

    //while(1){
    //    sleep(1);
    //}

    rotate_abs(d_yawx);
    if (dist != 0.0)
        go_abs(x,y,isBack);
}
/*
move_abs()
    x,y: 絶対番地への移動(基準座標)
    d_yaw: 基準座標での角度。 [degree]
*/
int RobotDriveCmd_Vel::move_abs(float x,float y,float d_yaw){
    rotate_abs(d_yaw);
    int rc=go_abs(x,y);
    return rc;
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
go_abs(x,y,isBack=false,speed=0.05)
直進する。
*/
int RobotDriveCmd_Vel::go_abs(float x,float y, bool isBack, bool obs_chk, float speed){

    std::cout << "C go_abs() isBack:" << isBack;

    float i_spped;
    int rc=0;

    // 前進です。
    if(!isBack)
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
    float prev_current_distance=0.0;    // add by nishi 2024.4.6

    std::cout << " start_distance=" << round_my<double>(start_distance,3) << std::endl;

    int i = 0;
    int j = 0;
    int chk_cnt=0;
    bool ex_f=false;

    unsigned char cost;
    float _rz_cur;
    // 前進です。
    if(!isBack)
        _rz_cur=_rz;
    else{
        // 反対を向ける
        _rz_cur=normalize_tf_rz(_rz+180.0/RADIANS_F);
    }

    // 監視 thread を起動 add by nishi 2024.9.4
    //std::thread th(&RobotDriveCmd_Vel::th_check_cource_obstacle, this, x ,y);

    black_cnt_=0;
    std::thread th;
    if((obs_chk==true || _dumper==true) && start_distance >= 0.9){
        th=std::thread(&RobotDriveCmd_Vel::th_check_cource_obstacle, this, x ,y);
    }

    //Loop to move the turtle in an specified distance
    while(current_distance < start_distance){
        //std::cout << "pub cmd_vel" << std::endl;
        //Publish the velocity
        _pub->publish(_vel_msg);

        rate.sleep();

        get_tf(0);

        tf2::Vector3 cur_origin = base_tf.getOrigin();

        //std::cout << "_x, _y,_z =" << cur_translation.getX() <<", "
        //  << cur_translation.getY() << " , " << cur_translation.getZ() << std::endl;

        float cur_x = cur_origin.getX();
        float cur_y = cur_origin.getY();

        off_x = cur_x - start_x;
        off_y = cur_y - start_y;
        off_z=0.0;

        current_distance = std::sqrt(off_x*off_x+off_y*off_y+off_z*off_z);

        // 前進です。
        if(!isBack)
            _rz_cur=_rz;
        // 後進です。
        else{
            // 反対を向ける
            _rz_cur=normalize_tf_rz(_rz+180.0/RADIANS_F);
        }

        // dumper ON
        //if(_dumper==true){
        //    cost=0;
        //    //cost=navi_.check_cost(cur_x,cur_y);
        //    if(cost>0){
        //        std::cout << "cost=" << (unsigned int)cost << std::endl;
        //        _vel_msg.linear.x = 0.0;
        //        // Force the robot to stop
        //        //_pub.publish(_vel_msg);
        //        _pub->publish(_vel_msg);
        //        //while(1){
        //        //    rate.sleep();
        //        //}
        //        return rc;
        //    }
        //}

        i+=1;
        if (i > 5){
            // ロボットが障害物に当たって止まっていないかチェック add by nishi 2024.4.6
            if(round_my<float>(prev_current_distance,2) == round_my<float>(current_distance,2)){
                chk_cnt++;
                if(chk_cnt > 3){
                    std::cout << " current_distance=" << round_my<float>(current_distance,3) << std::endl;
                    std::cout << " time out 1" << std::endl;
                    break;
                }
            }
            else
                chk_cnt=0;
            prev_current_distance = current_distance;

            // 自分からの目的地の方角
            float off_target_x = x - cur_x;
            float off_target_y = y - cur_y;

            // 前進も、後退もOK みたい
            // 後ろだと、後ろの角度が得られる。
            float theta_r = std::atan2(off_target_y,off_target_x);   //  [radian]
            // 自分の方向を減算
            float theta_ar = normalize_tf_rz(theta_r - _rz_cur);

            j++;
            if(j > 4){
                std::cout << " _dz_cur=" << round_my<double>(_rz_cur*RADIANS_F,3) <<" theta_d=" << round_my<float>(theta_r*RADIANS_F,3)
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
                        // for debug
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
            if(j > 4){
                std::cout << " current_distance=" << round_my<float>(current_distance,3) << std::endl;
                j=0;
            }
            i=0;

            // add by nishi 2024.9.5
            if(black_cnt_ > 4){
                std::cout << " found obstracle!!" << std::endl;
                rc=1;
                break;
            }
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

    //if((obs_chk==true || _dumper==true) && start_distance >= 1.0){
    if(th_check_cource_obstacle_f==true){
        th_check_cource_obstacle_f=false;
        th.join();
    }
    return rc;
}

/*
rotate_abs()
    stop_dz(d_theta) : [deg] 基本座標上の角度  > 0 左回転 /  < 0 右回転
    rad_f : false -> deg / true -> radian
    speed :  5.0  [deg/s]
*/
void RobotDriveCmd_Vel::rotate_abs(float stop_dz,bool rad_f, float speed){
//void RobotDriveCmd_Vel::rotate_abs(float stop_dz,float speed){
    // 目的の角度と速度を設定
    // stop_dz = 180.0 # [deg]
    // speed = 10.0 # [deg/s]


    //geometry_msgs::Twist _vel_msg;

    float stop_rz = stop_dz;
    if(!rad_f){
        stop_rz = stop_dz/ RADIANS_F;
    }

    // 小数点以下5 の 丸め
    stop_rz=round_my_zero(stop_rz);
    float stop_rz_origin = stop_rz;

    std::cout << "C rotate_abs() stop_dz=" << stop_rz*RADIANS_F << std::endl;

    _vel_msg.angular.x = _vel_msg.angular.y = _vel_msg.angular.z =0.0;
    _vel_msg.linear.x = _vel_msg.linear.y = _vel_msg.linear.z = 0.0;


    float rz_dlt = abs(speed / RADIANS_F) * 0.25;
    // #z_wind = abs(speed / RADIANS_F) * 2.0;
    float rz_wind = rz_dlt * 3.0;

    //# Twist 型のデータ
    //#t = Twist()
    //#t.linear.x = 0
    _vel_msg.linear.x = 0.0;

    get_tf(2);

    // 小数点以下5 の 丸め
    //_rz = round_my<float>(_rz,5);
    _rz=round_my_zero(_rz);

    std::cout << " _rx,_ry,_rz=" << _rx << ","<< _ry << "," << _rz << std::endl;

    int turn_plus=0;        // 1: turn left(+) , -1: turn right(-) , 0: 未定
    // 同じ回転空間で、目的角が少ない時は、回転向きを未定にする。
    if(stop_dz > 0){
        if(_rz > 0 && stop_rz > _rz)
            turn_plus=1;
    }
    else if(stop_dz < 0){
        if(_rz < 0 && stop_rz < _rz)
            turn_plus=-1;
    }

    // stop_dz 余り角
    float stop_dz_mod = fmod(stop_rz_origin*RADIANS_F,360.0);
    std::cout << " stop_dz_mod=" << stop_dz_mod  << std::endl;

    // stop_rz をノーマライズする。 180/RADIANS_F <= stop_rz <= -180/RADIANS_F
    int quo_i;
    stop_rz = normalize_tf_rz_quo(stop_rz,quo_i);

    // ロボットとの角度差を求める
    float r_theta = stop_rz - _rz;

    // add by nhishi 2024.4.7
    r_theta = normalize_tf_rz(r_theta);
    float tmp_r_theta = reverse_tf_rz(r_theta);
    // 移動角度の小さい方を採用する。
    if(abs(tmp_r_theta) < abs(r_theta))
        r_theta=tmp_r_theta;

    std::cout <<" turn_plus=" << turn_plus << " r_theta="<< r_theta << " quo_i=" << quo_i <<" stop_rz=" << stop_rz << std::endl;

    // turn_plus == 0 (未定) なら、 r_theta を回転方向とする。
    if(turn_plus == 0){
        if(r_theta >= 0)
            turn_plus = 1;
        else
            turn_plus = -1;
    }
    else{
        // 回転方向が違ったら、回転方向を反転する。
        if( (turn_plus == 1 && r_theta < 0) || (turn_plus == -1 && r_theta > 0)){
            std::cout <<" #3 passed "<<  std::endl;
            r_theta=reverse_tf_rz(r_theta);
        }
    }
    float r_theta_sav=r_theta;
    // quo の回転数を加味して、トータルの回転角度を計算
    r_theta = (float)quo_i * 360.0/RADIANS_F + r_theta;      // r_theta : トータルの回転角 [rad]

    std::cout << " d_theta=" << r_theta * RADIANS_F << std::endl;

    std::cout << " normalized stop_dz=" << stop_rz*RADIANS_F << " ,stop_dz_mod="<< stop_dz_mod << std::endl;

    if (abs(r_theta) <= rz_dlt)
        return;

    std::cout << " turn_plus=" << turn_plus << std::endl;

    // 反転時の角度差を検証する。
    float r_theta_reverse = reverse_tf_rz(r_theta);
    // 5.0 度以内なら、
    if(abs(r_theta) < 5.0/RADIANS_F || abs(r_theta_reverse) < 5.0/RADIANS_F){
        std::cout << " stop_dz is within 5.0[deg]" << std::endl;
        rotate_abs_179(stop_rz_origin, true,speed);
        return;
    }

    //# turn plus
    if (turn_plus >= 0){
        //#self.vel_msg.angular.z = speed * 3.1415 / 180.0 # [rad]
        //#self.vel_msg.angular.z = speed # [rad]
        _vel_msg.angular.z = speed / RADIANS_F;  // [rad]
    }
    // turn minus
    else{
        //#self.vel_msg.angular.z = speed * 3.1415 / -180.0 # [rad]
        _vel_msg.angular.z = speed / RADIANS_F * -1.0;  //  [rad]
    }

    // stop_rz を目指す
    rclcpp::WallRate rate(40);  // 40[Hz]
    float rz_min = 10.0;
    bool ok_f = false;

    int lp_cnt=0;
    int lp_cnt_chk=0;

    // 残り回転量をセット
    float remain_r_theta = r_theta;
    float prev_rz=_rz;  // 前回の回転位置を保存

    while(1){
        _pub->publish(_vel_msg);

        rate.sleep();
        rclcpp::spin_some(node_);

        get_tf(2);
        lp_cnt++;
        // 今回の回転量を計算します。 delta(角)を求める。
        float cur_rz = _rz - prev_rz;

        // TF _rz は、常に 0pi からの近い角度と向き(+-)が、返される。  +-|0pi <= _rz <= pi|
        // なので、 pi を跨った時の、delta(差)角 の計算には、補正が必要になる。
        // normalize_tf_rz() が、使えます。
        cur_rz=normalize_tf_rz(cur_rz);

        remain_r_theta -= cur_rz;

        if(lp_cnt > 20){
            std::cout << " remain_r_theta=" << round_my<float>(remain_r_theta,4) << " _rz="<< round_my<float>(_rz,4) << " cur_rz=" << round_my<float>(cur_rz,4)<< std::endl;
            lp_cnt=0;
        }

        // 残量角度 <= 30[rad] であれば、roate_abs_179() に任せる。
        if(abs(remain_r_theta) <= 30/RADIANS_F){
            rotate_abs_179(stop_dz_mod, false, speed);
            break;
        }

        // ロボットが、動障害物にぶつかって、動いていないかチェック
        //小数点以下 3桁は有効
        float cur_rz_x = round_my<float>(cur_rz,3);
        if(cur_rz_x == 0.0){
            lp_cnt_chk++;
            //if(lp_cnt_chk >= 9){
            // changed by nishi 2024.9.7
            if(lp_cnt_chk >= rotate_lp_cnt_chk_2_){
                std::cout << " time out 2" << std::endl;
                break;
            }
        }
        else{
            lp_cnt_chk=0;
        }
        prev_rz = _rz;

        //rate.sleep();
        //rclcpp::spin_some(node_);
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
rotate_abs_last179()
最後の 179[deg] を回転する
    stop_dz(d_theta) : [deg] 基本座標上の角度  > 0 左回転 /  < 0 右回転
    rad_f : false -> deg / true -> radian
    speed :  5.0  [deg/s]
    注)      speed >= 3.0
*/
void RobotDriveCmd_Vel::rotate_abs_179(float stop_dz,bool rad_f, float speed){
    // 目的の角度と速度を設定
    // stop_dz = 180.0 # [deg]
    // speed = 10.0 # [deg/s]
    bool speed_half_msg_f=false;

    int turn_plus=0;        // 1: turn left(+) , -1: turn right(-) , 0: 未定
    //if(stop_dz > 0)
    //    turn_plus=1;
    //else if(stop_dz < 0)
    //    turn_plus=-1;

    //geometry_msgs::Twist _vel_msg;

    float stop_rz = stop_dz;
    if(!rad_f){
        stop_rz = stop_dz/ RADIANS_F;
    }
    // 小数点以下5 の 丸め
    //stop_rz = round_my<float>(stop_rz,5);
    stop_rz=round_my_zero(stop_rz);

    std::cout << "C rotate_abs_179() stop_dz=" << stop_rz*RADIANS_F << " speed:" << speed << std::endl;
    if(speed < rotate_speed_min_){
        std::cout << " speed < "<< rotate_speed_min_<< " ,then set speed:" << rotate_speed_min_ << std::endl;
        speed = rotate_speed_min_;
    }

    _vel_msg.angular.x = _vel_msg.angular.y = _vel_msg.angular.z =0.0;
    _vel_msg.linear.x = _vel_msg.linear.y = _vel_msg.linear.z = 0.0;


    float rz_dlt = abs(speed / RADIANS_F) * 0.25;
    // #z_wind = abs(speed / RADIANS_F) * 2.0;
    float rz_wind = rz_dlt * 3.0;

    //# Twist 型のデータ
    //#t = Twist()
    //#t.linear.x = 0
    _vel_msg.linear.x = 0.0;

    get_tf(2);

    // 小数点以下5 の 丸め
    //_rz = round_my<float>(_rz,5);
    _rz=round_my_zero(_rz);

    std::cout << " _rx,_ry,_rz=" << _rx << ","<< _ry << "," << _rz << std::endl;

    // stop_rz をノーマライズする。 180/RADIANS_F <= stop_rz <= -180/RADIANS_F
    int quo_i;
    stop_rz = normalize_tf_rz_quo(stop_rz,quo_i);

    // ロボットとの角度差を求める
    float rz_off = stop_rz - _rz;

    // turn_plus == 0 (未定) なら、 r_theta を回転方向とする。
    if(turn_plus == 0){
        if(rz_off >= 0)
            turn_plus = 1;
        else
            turn_plus = -1;
    }
    else{
        // 回転方向が違ったら、回転方向を反転する。
        if( (turn_plus == 1 && rz_off < 0) || (turn_plus == -1 && rz_off > 0)){
            std::cout <<" #3 passed "<<  std::endl;
            rz_off=reverse_tf_rz(rz_off);
        }
    }

    // quo の回転数を加味して、トータルの回転角度を計算
    //rz_off = (float)quo_i * 360.0/RADIANS_F + rz_off;      // r_theta : トータルの回転角 [rad]

    if (abs(rz_off) <= rz_dlt)
        return;

    float stop_rz_norm = stop_rz;

    std::cout << " dz_off=" << rz_off * RADIANS_F << " stop_rz_norm=" << stop_rz_norm << std::endl;

    std::cout << " turn_plus=" << turn_plus << std::endl;

    //# turn plus
    if (turn_plus >= 0){
        //#self.vel_msg.angular.z = speed * 3.1415 / 180.0 # [rad]
        //#self.vel_msg.angular.z = speed # [rad]
        _vel_msg.angular.z = speed / RADIANS_F;  // [rad]
    }
    // turn minus
    else{
        //#self.vel_msg.angular.z = speed * 3.1415 / -180.0 # [rad]
        _vel_msg.angular.z = speed / RADIANS_F * -1.0;  //  [rad]
    }

    float speed_half = _vel_msg.angular.z * 0.5;
    if(abs(speed_half) < (speed / RADIANS_F)){
        if(speed_half >= 0.0)
             speed_half = speed / RADIANS_F;
        else
             speed_half = speed / RADIANS_F * -1.0;
    }

    if (abs(rz_off) <= 10.0/RADIANS_F){      //# 角度差が 10.0 度以内 であれば、補正回転にする
        std::cout << " adjust angle" << std::endl;
        _vel_msg.angular.z = speed_half;
    }
    else{
        std::cout << " non adjust angle" << std::endl;
    }

    std::cout << " turn_plus=" << turn_plus << std::endl;

    //print 'test1'
    //sys.exit()

    // stop_rz を目指す
    //ros::Rate rate(40);   // 40 [Hz]
    rclcpp::WallRate rate(40);
    float rz_min = 10.0;
    bool ok_f = false;
    int lp_cnt=0;

    int lp_cnt_chk=0;   // add by nishi 2024.9.7
    float prev_rz_off_round=100.0;    // add by nishi 2024.9.7

    while(1){
        _pub->publish(_vel_msg);

        get_tf(2);

        // 最も近い角度で終了します
        if (ok_f == true){
            std::cout << " ok nearly" << std::endl;
            break;
        }
        // 目的の角度です。
        //if (abs(_rz - stop_rz) <= rz_dlt){
        if (abs(stop_rz_norm - _rz) <= rz_dlt){
            //  std::cout << "ok just" << std::endl;
            //  break;
            // 目的の角度です。
            if (round_my<float>(stop_rz_norm,3) == round_my<float>(_rz,3)){
                std::cout << " ok just" << std::endl;
                break;
            }
        }
        //float rz_off = stop_rz - _rz;
        float rz_off = stop_rz_norm - _rz;
        // ノーマライズします。
        rz_off=normalize_tf_rz(rz_off);

        lp_cnt++;
        if(lp_cnt > 20){
            std::cout << " stop_rz_norm=" << round_my<float>(stop_rz_norm,3) << " rz_off="<< round_my<float>(rz_off,3) << " _rz=" << _rz << std::endl;
            lp_cnt=0;
        }

        if(rz_off > 0.0){
            if(_vel_msg.angular.z < 0.0)
                _vel_msg.angular.z *= -1.0;  // [rad]
        }
        // rz_off <  0  --->  regura clock(right) rotate(-)
        else{
            if(_vel_msg.angular.z > 0.0)
                _vel_msg.angular.z *= -1.0;  // [rad]
        }

        rz_off=abs(rz_off);
        if (rz_off <= 10.0/RADIANS_F){      //# 角度差が 10.0 度以内 であれば、補正回転にする
            if(speed_half_msg_f==false){
                _vel_msg.angular.z = speed_half;
                std::cout << " set speed_half" << std::endl;
                speed_half_msg_f=true;
            }
        }
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
        // ロボットの回転停止の監視 add by nishi 2024.9.7
        float off_round_x=round_my<float>(rz_off,3);
        if(prev_rz_off_round == off_round_x){
            lp_cnt_chk++;
            if(lp_cnt_chk > 20){
                std::cout << " time up!!" << std::endl;
                break;
            }
        }
        else{
            prev_rz_off_round=off_round_x;
            lp_cnt_chk=0;
        }
        rate.sleep();
        //ros::spinOnce();
        rclcpp::spin_some(node_);
    }
    std::cout << " stop" << std::endl;
    _vel_msg.angular.z = 0.0;   // [rad]
    //Force the robot to stop
    //_pub.publish(_vel_msg);
    _pub->publish(_vel_msg);
    rate.sleep();

    get_tf(2);
}

/* 
void rotate_off()
    d_theta : [deg] ロボットの今の向きからの角度   > 0 左回転 /  < 0 右回転
    speed :  5.0  [deg/s]
*/
void RobotDriveCmd_Vel::rotate_off(float d_theta, float speed, bool go_curve){
    // 目的の角度と速度を設定
    //d_theta = 180.0 # [deg]
    //speed = 10.0 # [deg/s]

    bool turn_plus;
    std::cout << "C rotate_off() d_theta:" << d_theta << std::endl;

    float rz_dlt = abs(speed/RADIANS_F) * 0.25;
    //float rz_wind = rz_dlt * 3.0;

    float r_theta = d_theta/RADIANS_F;
    if (abs(r_theta) <= rz_dlt)
        return;

    if (d_theta >= 0.0){
        turn_plus = true;   // reverse clock(left) rotate(+)
        //_vel_msg.angular.z = speed * 3.1415 / 180.0; //[rad]
        //_vel_msg.angular.z = speed; // [rad]
        _vel_msg.angular.z = speed/RADIANS_F;  // [rad]
    }
    else{
        turn_plus = false; // regura clock(right) rotate(-)
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

    // World 座標 stop_rz を目指す。
    float stop_rz =  normalize_tf_rz(r_theta+_rz);

    rclcpp::WallRate rate(40);  // 40[Hz]
    //float rz_min = 10.0;

    bool ok_f = false;

    int lp_cnt=0;
    int lp_cnt_chk=0;

    // 残り回転量をセット
    float remain_r_theta = r_theta;
    float prev_rz=_rz;  // 前回の回転位置を保存

    while(1){
        _pub->publish(_vel_msg);

        rate.sleep();
        rclcpp::spin_some(node_);

        get_tf(2);
        lp_cnt++;
        // 今回の回転量を計算します。 delta(角)を求める。
        float cur_rz = _rz - prev_rz;

        // TF _rz は、常に 0pi からの近い角度と向き(+-)が、返される。  +-|0pi <= _rz <= pi|
        // なので、 pi を跨った時の、delta(差)角 の計算には、補正が必要になる。
        #if defined(TEST_KYK)
        // pi を跨った計算です。 left(+) 角値  から right(-) 角値 迄の delta(差)角 計算
        if(cur_rz < -180/RADIANS_F)
            // delta = 2pi - delta
            cur_rz = 360/RADIANS_F - cur_rz;
        // pi を跨った計算です。 right(-) 角値 から left(+) 角値 迄の delta(差)角 計算
        else if(cur_rz > 180/RADIANS_F)
            // delta = delta - 2pi
            cur_rz = cur_rz - 360/RADIANS_F;
        #endif
        // normalize_tf_rz() が、使えます。
        cur_rz=normalize_tf_rz(cur_rz);

        remain_r_theta -= cur_rz;

        if(lp_cnt > 40){
            std::cout << " remain_r_theta=" << round_my<float>(remain_r_theta,4) << " _rz="<< round_my<float>(_rz,4) << " cur_rz=" << round_my<float>(cur_rz,4)<< std::endl;
            lp_cnt=0;
        }

        // 残量角度 <= 30[rad] であれば、roate_abs_179() に任せる。
        if(abs(remain_r_theta) <= 30/RADIANS_F){
            rotate_abs_179(stop_rz, true, speed);
            break;
        }
        // ロボットが、動障害物にぶつかって、動いていないかチェック
        //小数点以下 3桁は有効
        float cur_rz_x = round_my<float>(cur_rz,3);
        if(cur_rz_x == 0.0){
            lp_cnt_chk++;
            //if(lp_cnt_chk >= 6){
            if(lp_cnt_chk >= 12){
                std::cout << " time out 3" << std::endl;
                break;
            }
        }
        else{
            lp_cnt_chk=0;
        }
        prev_rz = _rz;

        //rate.sleep();
        //rclcpp::spin_some(node_);
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




