/*
* Programable Controller Core
*  turtlebot3_navi_my/src/pro_control.cpp
*
* https://docs.ros2.org/latest/api/rclcpp/classrclcpp_1_1Node.html
* https://answers.ros.org/question/307370/ros2-whats-the-best-way-to-wait-for-a-new-message/
* https://github.com/ros-planning/navigation2/blob/main/nav2_map_server/src/map_saver/map_saver.cpp
*/

#include "turtlebot3_navi_my/pro_control.hpp"
//#include <cmath.h>


/*
* Programable Controller Core Class
*  class ProControl
*/
void ProControl::init(std::shared_ptr<rclcpp::Node> node, bool use_costmap){

    std::cout << "ProControlCore::init():#1 " << std::endl;

    node_=node;

    use_costmap_=use_costmap;

	get_map_func_ = node_->declare_parameter<int>("get_map_func", get_map_func_);

    #if defined(USE_NAV2)
        //drive_=&drive_nav; 
        // Sub1* sub1 = dynamic_cast<Sub1*>(new Base());  // ダウンキャスト
        //drive_=dynamic_cast<Robot_DriveCore *>(new RobotDriveNAV2());     
        drive_=dynamic_cast<Robot_DriveCore *>(&drive_nav);     

        drive_nav.init(node,&getTF,true);   
        drive_cmd.init(node_,&getTF,true);                // changed by nishi 2024.2.27
        mode_f_origin=mode_f=1;    // 1: navigation2 mode
    #else
        //drive_=&drive_cmd;
        //drive_=dynamic_cast<Robot_DriveCore *>(new RobotDriveCmd_Vel());     
        drive_=dynamic_cast<Robot_DriveCore *>(&drive_cmd);     
        drive_cmd.init(node_,&getTF,true);                // changed by nishi 2024.2.27
        mode_f_origin=mode_f=0;       // 0:cmd_vel mode
    #endif

    std::cout << "ProControl::init():#1.1 mode_f="<< mode_f << std::endl;

    get_map.init(node,get_map_func_);
    // add by nishi 2024.9.1
    if(use_costmap_==true){
        //get_costmap.init(node,get_map_func_,"local_costmap/costmap");
        get_costmap.init(node,get_map_func_,"local_costmap/costmap",false); // changed by nishi 2024.9.26
    }

    // add by nishi 2024.9.4
    drive_cmd.set_map(&get_costmap);

    std::cout << "ProControl::init():#2 " << std::endl;

    goalId = 0;
    sts=0;
    t_type=0;

    force_start_origin=false;

    start_pos_x = start_pos_y = start_pos_z = 0.0;
    start_rot_rz = 0.0;  // add by nishi 2022.11.15 by nishi

    sleep(1);
}



/*
mloop_ex(GoalList *goalList)
    goalList: ゴールリスト
*/
void ProControl::mloop_ex(GoalList *goalList){
    // params & variables
    _goalList = goalList;
    goalId = 0;
    t_type=0;
    mloop();
}
/*
mloop_ex2(GoalList2 *goalList2)
    goalList: ゴールリスト2
*/
void ProControl::mloop_ex2(GoalList2 *goalList2){
    // params & variables
    _goalList2 = goalList2;
    goalId = 0;
    t_type=1;
    mloop();
}

/*
* set_drive_mode(int func)
* func: 0 -> cmd_vel, 1 -> navi2
*/
void ProControl::set_drive_mode(int func){
    #if defined(USE_NAV2)
        if(func==1){
            if(mode_f_origin ==1 && mode_f==0){
                drive_=dynamic_cast<Robot_DriveCore *>(&drive_nav);
                mode_f=1;
            }
        }
        else{
            if(mode_f_origin ==1 && mode_f==1){
                drive_=dynamic_cast<Robot_DriveCore *>(&drive_cmd);
                mode_f=0;
            }
        }
    #endif
}

/*
* check_obstacle_backaround(float r_lng,int black_thresh)
*  float r_lng: ロボットの周囲の半径[M]
*  int black_thresh: black count の閾値
*  ロボットの前方の障害物をチェックして、前方が塞がっていれば、後ろ向きにする。
*/
void ProControl::check_obstacle_backaround(float r_lng,int black_thresh){
    get_map.get();
    drive_->get_tf(2);  
    tf2::Vector3 cur_origin = drive_->base_tf.getOrigin();  // changed by nishi 2024.2.28

    float cur_x_tmp = cur_origin.getX();    // World point(基本座標) [M]
    float cur_y_tmp = cur_origin.getY();    // World point(基本座標) [M]
    // ロボットの今の向き
    std::cout << "drive->_rz*RADIANS_F: " << drive_->_rz*RADIANS_F <<std::endl;
    // 前方チェック
    int rcx= get_map.check_obstacle(cur_x_tmp,cur_y_tmp,drive_->_rz,r_lng,0,black_thresh);
    if(rcx!=0){
        set_drive_mode(0);      // set cmd_vel mode
        // 後ろを向かせる
        drive_->rotate_off(90.0);   // changed by nishi 2024.3.7
        drive_->rotate_off(90.0);   // changed by nishi 2024.3.7

        // 再度前方チェックして、障害物がなければ、0.05[M] 前に移動

        get_map.get();
        drive_->get_tf(2);  
        cur_origin = drive_->base_tf.getOrigin();  // changed by nishi 2024.2.28

        cur_x_tmp = cur_origin.getX();    // World point(基本座標) [M]
        cur_y_tmp = cur_origin.getY();    // World point(基本座標) [M]
        // 前方チェック
        rcx= get_map.check_obstacle(cur_x_tmp,cur_y_tmp,drive_->_rz,r_lng,0,black_thresh);
        if(rcx==0){
            std::cout << "go forward 0.1[M]" <<std::endl;
            drive_->move(0.1,0.0);
        }
        set_drive_mode(1);      // set nav2 mode
    }
    else{
        //後ろをチェック
        rcx= get_map.check_obstacle(cur_x_tmp,cur_y_tmp,drive_->_rz,r_lng,2,black_thresh);
        if(rcx!=0){
            // 前に動かす
            std::cout << "go forward 0.1[M]" <<std::endl;
            set_drive_mode(0);      // set cmd_vel mode
            drive_->move(0.1,0.0);
            set_drive_mode(1);      // set nav2 mode
        }
    }
}

/*
* obstacle_escape(float r_lng,int black_thresh,float move_l)
*  float r_lng: ロボットの周囲の半径[M] = 0.6
*  int black_thresh: black count の閾値
*  float move_l: 移動距離[M] =0.12
*  ロボットの四方の障害物をチェックして、障害物から少しだけ、離れる。
*/
void ProControl::obstacle_escape(float r_lng,int black_thresh,float move_l){
    bool all_black=true;
    GetMap *get_map_r;
    // add by nishi 2024.9.25
    // local_costmap を使う!!
    if(use_costmap_==true)
        get_map_r = &get_costmap;
    else
        get_map_r = &get_map;
    
    // static map or cost map with global_frame: map
    if(get_map_r->map_orient_fix_==true){
        obstacle_escape_map_orient(r_lng, black_thresh, move_l, get_map_r);
    }
    // cost map with global_frame: base_footprint
    else{
        obstacle_escape_robo_orient(r_lng, black_thresh, move_l, get_map_r);
    }
}

/*
* obstacle_escape_map_orient(float r_lng,int black_thresh,float move_l,GetMap *get_map_r)
*
*  static map or cost map with global_frame: map の時に実行します。
*
*  float r_lng: ロボットの周囲の半径[M] = 0.6
*  int black_thresh: black count の閾値
*  float move_l: 移動距離[M] =0.12
*  ロボットの四方の障害物をチェックして、障害物から少しだけ、離れる。
*  static map or cost map の方角が、tf-map と同じ。
*   向きは、常に、tf-map(0,0) の向きと同じになります。
*/
void ProControl::obstacle_escape_map_orient(float r_lng,int black_thresh,float move_l,GetMap *get_map_r){

    std::cout << "ProControl::obstacle_escape_map_orient() called" <<std::endl;

    get_map_r->get();
    drive_->get_tf(2);  
    tf2::Vector3 cur_origin = drive_->base_tf.getOrigin();  // changed by nishi 2024.2.28

    float cur_x_tmp = cur_origin.getX();    // World point(基本座標) [M]
    float cur_y_tmp = cur_origin.getY();    // World point(基本座標) [M]
    // ロボットの今の向き
    std::cout << " drive->_rz*RADIANS_F: " << drive_->_rz*RADIANS_F <<std::endl;

    int black_counts[4];    // 前、後ろ、左、右
    int min_black_count=1000;
    int min_black_idx=-1;
    int slant_black_counts[4];  // 斜め前、斜め後ろ、斜め左、斜め後ろ add by nishi 2024.4.8
    int slant_min_black_count=1000; // add by nhishi 2024.4.8
    int slant_min_black_idx=-1;     // add by nhishi 2024.4.8
    // 全方向をチェックします。 前、後ろ、左、右
    for(int i=0;i<4;i++){
        black_counts[i]= get_map_r->check_obstacle(cur_x_tmp,cur_y_tmp,drive_->_rz,r_lng,i,0);
        if(black_counts[i] < min_black_count){
            min_black_count=black_counts[i];
            min_black_idx=i;
        }
    }
    // 45度ずらして、斜め方向をチェックする。
    float slant_rz = normalize_tf_rz(drive_->_rz+45/RADIANS_F);


    set_drive_mode(0);      // set cmd_vel mode

    // 全方向は、障害物が無い。
    if(min_black_count <= black_thresh){
        // 前方向は、障害物無し
        if(black_counts[0] <= black_thresh){
            std::cout << "stay forward" <<std::endl;
            // 後ろ方向に障害物があります。
            if(black_counts[2] > black_thresh){
                // 前に、0.1[M] 動かす。
                std::cout << "go forward "<<move_l <<"[M]" <<std::endl;
                drive_->move(move_l,0.0);
            }
            // 左右に障害物があります。
            else if((black_counts[1] > 0 && black_counts[3] > 0) || (black_counts[1] > black_thresh) || (black_counts[3] > black_thresh)){
                // 前に、0.1[M] 動かす。
                std::cout << "go forward "<<move_l <<"[M]" <<std::endl;
                drive_->move(move_l,0.0);
            }
        }
        // 後ろの障害物が、有りません。
        else if(black_counts[2] <= black_thresh){
            // 後ろへ 0.12[M] 動かす。
            std::cout << "go backward "<<move_l <<"[M]" <<std::endl;
            drive_->move(-move_l,0.0);

            std::cout << "turn around 180" <<std::endl;
            // 後ろを向かせる
            drive_->rotate_off(180.0);
        }
        // 左が、右より障害物が少ない
        else if(black_counts[1] < black_counts[3]){
            // 左が障害物が無い
            if(black_counts[1] <= black_thresh){
                std::cout << "turn left" <<std::endl;
                //左へ向かせる。
                drive_->rotate_off(90.0);
                // 右が、障害物あり
                if(black_counts[3] > black_thresh){
                    // 前に、0.1[M] 動かす。
                    std::cout << "go leftward "<<move_l <<"[M]" <<std::endl;
                    drive_->move(move_l,0.0);
                }
            }
        }
        // 右が、障害物が無い
        else if(black_counts[3] <= black_thresh){
            std::cout << "turn right" <<std::endl;
            //右へ向かせる。
            drive_->rotate_off(-90.0);
            // 左は、障害物がある。
            if(black_counts[1] > black_thresh){
                // 前に、0.1[M] 動かす。
                std::cout << "go rightward "<<move_l <<"[M]" <<std::endl;
                drive_->move(move_l,0.0);
            }
        }
    }
    else{
        // 斜め方向をチェックします。 斜め前、斜め後ろ、斜め左、斜め後ろ add by nishi 2024.4.8
        for(int i=0;i<4;i++){
            slant_black_counts[i]= get_map_r->check_obstacle(cur_x_tmp,cur_y_tmp,slant_rz,r_lng,i,0);
            if(slant_black_counts[i] < slant_min_black_count){
                slant_min_black_count=slant_black_counts[i];
                slant_min_black_idx=i;
            }
        }

        // 斜め方向は、障害物が無い。
        if(slant_min_black_count <= black_thresh){
            switch(slant_min_black_idx){
                case 0: // 斜め前方が空いている
                    // 斜め前に、0.12[M] 動かす。
                    std::cout << " go slant forward "<<move_l <<"[M]" <<std::endl;
                    //左へ45向かせる。
                    drive_->rotate_off(45.0);
                    drive_->move(move_l,0.0);
                break;
                case 1:
                    //斜め左へ向かせる。
                    drive_->rotate_off(90.0+45.0);
                    // 前に、0.02[M] 動かす。
                    std::cout << " go slant leftward "<<move_l <<"[M]" <<std::endl;
                    drive_->move(move_l,0.0);
                break;
                case 2:
                    // 斜め後ろに退避
                    std::cout << " go slant back "<<move_l <<"[M] and turn around 180" <<std::endl;
                    drive_->rotate_off(45.0);
                    drive_->move(-move_l,0.0);
                    // 斜め後ろを向かせる
                    //drive_->rotate_off(90.0);
                    drive_->rotate_off(180.0);
                break;
                case 3:
                    //斜め右へ向かせる。
                    drive_->rotate_off(45.0-90.0);
                    // 前に、0.02[M] 動かす。
                    std::cout << " go slant rightward "<<move_l <<"[M]" <<std::endl;
                    drive_->move(move_l,0.0);
                break;
            }        
        }
        else{
            std::cout << "all closed" <<std::endl;
            // 実機だと、うごかすのは危険か?
            // 一番空いている方へ移動させる。
            switch(min_black_idx){
                case 0: // 前方が空いている
                    // 前に、0.02[M] 動かす。
                    std::cout << " go forward 0.04[M]" <<std::endl;
                    drive_->move(0.04,0.0);
                break;
                case 1:
                    //左へ向かせる。
                    drive_->rotate_off(90.0);
                    // 前に、0.02[M] 動かす。
                    std::cout << " go leftward 0.04[M]" <<std::endl;
                    drive_->move(0.04,0.0);
                break;
                case 2:
                    std::cout << " go back 0.04[M] and turn around 180" <<std::endl;
                    drive_->move(-0.04,0.0);
                    // 後ろを向かせる
                    //drive_->rotate_off(90.0);
                    drive_->rotate_off(180.0);
                break;
                case 3:
                    //右へ向かせる。
                    drive_->rotate_off(-90.0);
                    // 前に、0.02[M] 動かす。
                    std::cout << " go rightward 0.04[M]" <<std::endl;
                    drive_->move(0.04,0.0);
                break;
            }
        }
    }
    set_drive_mode(1);      // set nav2 mode
}

/*
* obstacle_escape_robo_orient(float r_lng,int black_thresh,float move_l,GetMap *get_map_r)
*
*  cost map with global_frame: base_footprint の時に実行します。
*
*  float r_lng: ロボットの周囲の半径[M] = 0.6
*  int black_thresh: black count の閾値
*  float move_l: 移動距離[M] =0.12
*  ロボットの四方の障害物をチェックして、障害物から少しだけ、離れる。
*
*  cost map の方角が、robot と同じ向きになる。robot の動きに連動します。
*/
void ProControl::obstacle_escape_robo_orient(float r_lng,int black_thresh,float move_l,GetMap *get_map_r){

    std::cout << "ProControl::obstacle_escape_robo_orient() called" <<std::endl;

    get_map_r->get();
    drive_->get_tf(2);  
    //tf2::Vector3 cur_origin = drive_->base_tf.getOrigin();  // changed by nishi 2024.2.28

    //float cur_x_tmp = cur_origin.getX();    // World point(基本座標) [M]
    //float cur_y_tmp = cur_origin.getY();    // World point(基本座標) [M]
    float cur_x_tmp = 0.0;    // World point(基本座標) [M]
    float cur_y_tmp = 0.0;    // World point(基本座標) [M]
    // ロボットの今の向き
    std::cout << " drive->_rz*RADIANS_F: " << drive_->_rz*RADIANS_F <<std::endl;

    double rz=0.0;

    int black_counts[4];    // 前、後ろ、左、右
    int min_black_count=1000;
    int min_black_idx=-1;
    int slant_black_counts[4];  // 斜め前、斜め後ろ、斜め左、斜め後ろ add by nishi 2024.4.8
    int slant_min_black_count=1000; // add by nhishi 2024.4.8
    int slant_min_black_idx=-1;     // add by nhishi 2024.4.8
    // 全方向をチェックします。 前、後ろ、左、右
    for(int i=0;i<4;i++){
        //black_counts[i]= get_map_r->check_obstacle(cur_x_tmp,cur_y_tmp,drive_->_rz,r_lng,i,0);
        black_counts[i]= get_map_r->check_obstacle(cur_x_tmp,cur_y_tmp,rz,r_lng,i,0);
        if(black_counts[i] < min_black_count){
            min_black_count=black_counts[i];
            min_black_idx=i;
        }
    }
    // 45度ずらして、斜め方向をチェックする。
    //float slant_rz = normalize_tf_rz(drive_->_rz+45/RADIANS_F);
    float slant_rz = normalize_tf_rz(rz+45/RADIANS_F);


    set_drive_mode(0);      // set cmd_vel mode

    // 全方向は、障害物が無い。
    if(min_black_count <= black_thresh){
        // 前方向は、障害物無し
        if(black_counts[0] <= black_thresh){
            std::cout << "stay forward" <<std::endl;
            // 後ろ方向に障害物があります。
            if(black_counts[2] > black_thresh){
                // 前に、0.1[M] 動かす。
                std::cout << "go forward "<<move_l <<"[M]" <<std::endl;
                drive_->move(move_l,0.0);
            }
            // 左右に障害物があります。
            else if((black_counts[1] > 0 && black_counts[3] > 0) || (black_counts[1] > black_thresh) || (black_counts[3] > black_thresh)){
                // 前に、0.1[M] 動かす。
                std::cout << "go forward "<<move_l <<"[M]" <<std::endl;
                drive_->move(move_l,0.0);
            }
        }
        // 後ろの障害物が、有りません。
        else if(black_counts[2] <= black_thresh){
            // 後ろへ 0.12[M] 動かす。
            std::cout << "go backward "<<move_l <<"[M]" <<std::endl;
            drive_->move(-move_l,0.0);

            std::cout << "turn around 180" <<std::endl;
            // 後ろを向かせる
            drive_->rotate_off(180.0);
        }
        // 左が、右より障害物が少ない
        else if(black_counts[1] < black_counts[3]){
            // 左が障害物が無い
            if(black_counts[1] <= black_thresh){
                std::cout << "turn left" <<std::endl;
                //左へ向かせる。
                drive_->rotate_off(90.0);
                // 右が、障害物あり
                if(black_counts[3] > black_thresh){
                    // 前に、0.1[M] 動かす。
                    std::cout << "go leftward "<<move_l <<"[M]" <<std::endl;
                    drive_->move(move_l,0.0);
                }
            }
        }
        // 右が、障害物が無い
        else if(black_counts[3] <= black_thresh){
            std::cout << "turn right" <<std::endl;
            //右へ向かせる。
            drive_->rotate_off(-90.0);
            // 左は、障害物がある。
            if(black_counts[1] > black_thresh){
                // 前に、0.1[M] 動かす。
                std::cout << "go rightward "<<move_l <<"[M]" <<std::endl;
                drive_->move(move_l,0.0);
            }
        }
    }
    else{
        // 斜め方向をチェックします。 斜め前、斜め後ろ、斜め左、斜め後ろ add by nishi 2024.4.8
        for(int i=0;i<4;i++){
            slant_black_counts[i]= get_map_r->check_obstacle(cur_x_tmp,cur_y_tmp,slant_rz,r_lng,i,0);
            if(slant_black_counts[i] < slant_min_black_count){
                slant_min_black_count=slant_black_counts[i];
                slant_min_black_idx=i;
            }
        }

        // 斜め方向は、障害物が無い。
        if(slant_min_black_count <= black_thresh){
            switch(slant_min_black_idx){
                case 0: // 斜め前方が空いている
                    // 斜め前に、0.12[M] 動かす。
                    std::cout << " go slant forward "<<move_l <<"[M]" <<std::endl;
                    //左へ45向かせる。
                    drive_->rotate_off(45.0);
                    drive_->move(move_l,0.0);
                break;
                case 1:
                    //斜め左へ向かせる。
                    drive_->rotate_off(90.0+45.0);
                    // 前に、0.02[M] 動かす。
                    std::cout << " go slant leftward "<<move_l <<"[M]" <<std::endl;
                    drive_->move(move_l,0.0);
                break;
                case 2:
                    // 斜め後ろに退避
                    std::cout << " go slant back "<<move_l <<"[M] and turn around 180" <<std::endl;
                    drive_->rotate_off(45.0);
                    drive_->move(-move_l,0.0);
                    // 斜め後ろを向かせる
                    //drive_->rotate_off(90.0);
                    drive_->rotate_off(180.0);
                break;
                case 3:
                    //斜め右へ向かせる。
                    drive_->rotate_off(45.0-90.0);
                    // 前に、0.02[M] 動かす。
                    std::cout << " go slant rightward "<<move_l <<"[M]" <<std::endl;
                    drive_->move(move_l,0.0);
                break;
            }        
        }
        else{
            std::cout << "all closed" <<std::endl;
            // 実機だと、うごかすのは危険か?
            // 一番空いている方へ移動させる。
            switch(min_black_idx){
                case 0: // 前方が空いている
                    // 前に、0.02[M] 動かす。
                    std::cout << " go forward 0.04[M]" <<std::endl;
                    drive_->move(0.04,0.0);
                break;
                case 1:
                    //左へ向かせる。
                    drive_->rotate_off(90.0);
                    // 前に、0.02[M] 動かす。
                    std::cout << " go leftward 0.04[M]" <<std::endl;
                    drive_->move(0.04,0.0);
                break;
                case 2:
                    std::cout << " go back 0.04[M] and turn around 180" <<std::endl;
                    drive_->move(-0.04,0.0);
                    // 後ろを向かせる
                    //drive_->rotate_off(90.0);
                    drive_->rotate_off(180.0);
                break;
                case 3:
                    //右へ向かせる。
                    drive_->rotate_off(-90.0);
                    // 前に、0.02[M] 動かす。
                    std::cout << " go rightward 0.04[M]" <<std::endl;
                    drive_->move(0.04,0.0);
                break;
            }
        }
    }
    set_drive_mode(1);      // set nav2 mode
}


/*
move_abs_auto_select()
    走行コースの、障害物を判定して、cmd_vel と nav2 を選択して、 move_abs() を実行する。
    x,y: 絶対番地への移動(基準座標)
    r_yaw: 基準座標での角度。 [radian]
    robo_radian_marker: ロボットの半径 [M] =0.2
*/
bool ProControl::move_abs_auto_select(float x,float y,float r_yaw,float robo_radian_marker){
    std::cout << "ProControl::move_abs_auto_select() called"<< std::endl;
    std::cout << " x:"<< x << " y:" << y << " d_yaw:" << r_yaw*RADIANS_F << std::endl;
    //rotate_f=false;
    bool rc=true;

    // original nav2 mode
    if(get_map.get() != true)
    {
        std::cout << "  get_map error occured , then move_abs_auto_select() is not executable!!" << std::endl;
        return false;
    }
    //if(get_costmap.get() != true)
    //{
    //    std::cout << "  get_costmap error occured , then move_abs_auto_select() is not executable!!" << std::endl;
    //    return -1;
    //}

    drive_->get_tf(2);
    tf2::Vector3 start_origin = drive_->base_tf.getOrigin();

    float cur_x = start_origin.getX();
    float cur_y = start_origin.getY();

    // 自分からの目的地の方角
    float off_target_x = x - cur_x;
    float off_target_y = y - cur_y;
    // 目的地の方角
    float theta_r = std::atan2(off_target_y,off_target_x);   //  [radian]
    theta_r = normalize_tf_rz(theta_r);
    // 反転時の角度差を検証する。
    float tmp_theta_r = reverse_tf_rz(theta_r);
    // 移動角度の小さい方を採用する。
    if(abs(tmp_theta_r) < abs(theta_r))
        theta_r=tmp_theta_r;

    std::cout << " theta_d:"<< theta_r*RADIANS_F << std::endl;

    // original cmd_vel mode?
    if(mode_f_origin == 0){
        std::cout << " #1 select drive_cmd" << std::endl;
        drive_cmd.rotate_abs(theta_r,true);
        drive_cmd.go_abs(x,y);
        drive_cmd.rotate_abs(r_yaw,true);
        return true;
    }

    // (cur_x,cur_y) -> (x,y) 間のロボット幅+α の障害物をチェック
    //float robo_radian=0.2;
    if(get_map.cource_obstacle_eye(cur_x,cur_y,x,y,robo_radian_marker,0)==0){
    //if(check_cource_obstacle_comb(get_map,get_costmap,cur_x,cur_y,x,y,robo_radian,0)==0){
        std::cout << " #2 select drive_cmd" << std::endl;

        drive_cmd.rotate_abs(theta_r,true);
        // check obostacle changed by nishi 2024.9.4
        if(drive_cmd.go_abs(x, y, false, true)==1){
            #if defined(USE_NAV2)
                std::cout << " #2.1 change drive_nav" << std::endl;
                rc=drive_nav.navi_move(x,y,r_yaw);
            #endif
        }
        else{
            drive_cmd.rotate_abs(r_yaw,true);
        }
    }
    else{
        #if defined(USE_NAV2)
            std::cout << " #3 select drive_nav" << std::endl;
            rc=drive_nav.navi_move(x,y,r_yaw);
        #else
        std::cout << " #4 select drive_cmd" << std::endl;
            drive_cmd.rotate_abs(theta_r,true);
            drive_cmd.go_abs(x,y);
            drive_cmd.rotate_abs(r_yaw,true);
        #endif
    }
    return rc;
}


/*
move_abs_auto_select_check()
    走行コースの、障害物を判定して、cmd_vel と nav2 のどちらを選択するか判定する。
    x,y: 絶対番地への移動(基準座標)
    r_yaw: 基準座標での角度。 [radian]
    robo_radian_marker: ロボットの半径 [M] = 0.2
    rc:
        0 : cmd_vel
        1 : navi
       -1 : error
*/
int ProControl::move_abs_auto_select_check(float x,float y,float r_yaw,float robo_radian_marker){ // add by nishi 2024.4.7
    std::cout << "ProControl::move_abs_auto_select_check() called"<< std::endl;
    std::cout << " x:"<< x << " y:" << y << " d_yaw:" << r_yaw*RADIANS_F << std::endl;
    //rotate_f=false;
    int rc=0;

    // original nav2 mode
    if(get_map.get() != true)
    {
        std::cout << "  get_map error occured , then move_abs_auto_select_check() is not executable!!" << std::endl;
        return -1;
    }
    //if(get_costmap.get() != true)
    //{
    //    std::cout << "  get_costmap error occured , then move_abs_auto_select_check() is not executable!!" << std::endl;
    //    return -1;
    //}

    drive_->get_tf(2);
    tf2::Vector3 start_origin = drive_->base_tf.getOrigin();

    float cur_x = start_origin.getX();
    float cur_y = start_origin.getY();

    // 自分からの目的地の方角
    float off_target_x = x - cur_x;
    float off_target_y = y - cur_y;
    // 目的地の方角
    float theta_r = std::atan2(off_target_y,off_target_x);   //  [radian]
    theta_r = normalize_tf_rz(theta_r);
    // 反転時の角度差を検証する。
    float tmp_theta_r = reverse_tf_rz(theta_r);
    // 移動角度の小さい方を採用する。
    if(abs(tmp_theta_r) < abs(theta_r))
        theta_r=tmp_theta_r;

    std::cout << " theta_d:"<< theta_r*RADIANS_F << std::endl;

    // original cmd_vel mode?
    if(mode_f_origin == 0){
        std::cout << " #1 select drive_cmd" << std::endl;
        return rc;
    }

    // (cur_x,cur_y) -> (x,y) 間のロボット幅+α の障害物をチェック
    //float robo_radian=0.2;
    if(get_map.cource_obstacle_eye(cur_x,cur_y,x,y,robo_radian_marker,0)==0){
    //if(check_cource_obstacle_comb(get_map,get_costmap,cur_x,cur_y,x,y,robo_radian,0)==0){
        std::cout << " #2 select drive_cmd" << std::endl;
        return rc;
    }
    else{
        #if defined(USE_NAV2)
            rc=1;
            std::cout << " #3 select drive_nav" << std::endl;
        #else
            std::cout << " #4 select drive_cmd" << std::endl;
        #endif
    }
    return rc;
}


/*
move(self,dist,deg)
    dist: 移動距離
    deg: 方向 [度]
*/
//void ProControl::move(self,dist,deg){
//    goalId = 0;
//    y = dist * math.sin(math.radians(deg)) + self.goalMsg.pose.position.y;
//    x = dist * math.cos(math.radians(deg)) + self.goalMsg.pose.position.x;
//    oz = math.radians(deg);
//    self.goalList = [[0, x,y, oz]];
//    mloop();
//}

/*
m_move(self,m_list)
    m_list
    [[func,dist,deg],...]
    func: 0 -> move dist,deg
            1 -> sleep
            2 -> get map
            50 -> set Navigation mode
            99 -> end
*/
//void ProControl::m_move(self,m_list){
//    int cur=0;
//    while(1):
//        if (cur >= len(m_list))
//            break;
//        if (m_list[cur][0] == 9)
//            break;
//        if (m_list[cur][0] != 0){
//            self.goalId = 0;
//            self.goalList = [[m_list[cur][0], 0, 0]];
//            mloop();
//        }
//        else{
//            dist = m_list[cur][1];
//            deg = m_list[cur][2];
//            move(dist,deg);
//        }
//        cur += 1;
//}


/*
mloop(self)
    self.goalList =[[func,x,y,d_yaw],....] or [[func,dist,d_yaw],....]
    func,x,y,d_yaw
        func: 0 -> rc=move_abs(x, y,d_yaw) : roate_abs(d_yaw) and go_abs(x, y)
            1 -> rc=go_abs(x, y)
            2 -> rotate_abs(d_yaw)
            3 -> roate_off(d_yaw)
            10 -> navi_move(x, y, d_yaw)
            11 -> move_abs_auto_select(x,y,d_yaw) // add by nishi 2024.4.7
            12 -> ProControl::obstacle_escape(float r_lng,int black_thresh,float move_l) // add by nishi 2024.9.28

    func,dist,d_yaw
        func: 0 -> move dist and rotate d_yaw

    func
            21 -> sleep
            22 -> get map
            23 -> map update
            30 -> auto map build
            31 -> auto map build of anchor
            35 -> auto mower
            36 -> auto mower2
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
            82 -> disable error auto stop
            99 -> end
*/
void ProControl::mloop(){
    bool f=true;
    u_int8_t func;
    int rc;
    while(f){
        if(t_type==0)
            func = _goalList[goalId].func;
        else
            func = _goalList2[goalId].func;

        switch (func){
            case 0:
            case 1:
            case 2:
            case 3:
            case 10:
            case 11:        // add by nishi 2024.4.7
            case 12:        // add by nishi 2024.9.28 for obstacle_escape test
                rc=mloop_sub();
                if(rc != 0 && error_auto_stop_==true){
                    std::cout << " stop moving by obstacle" << std::endl;
                    f=false;
                }
                break;
            case 21:
                // sleep
                sleep(1);
                break;
            case 22:
                get_map.get(true);
                // 障害物の判定テストです。
                //#define TEST_KK_888
                #if defined(TEST_KK_888)
                {
                    drive_->get_tf(2);      // changed by nishi 2024.2.28
                    tf2::Vector3 cur_origin = drive_->base_tf.getOrigin();  // changed by nishi 2024.2.28

                    float cur_x = cur_origin.getX();    // World point(基本座標) [M]
                    float cur_y = cur_origin.getY();    // World point(基本座標) [M]
                    // ロボットの今の向き
                    std::cout << "drive->_rz*RADIANS_F: " << drive_->_rz*RADIANS_F <<std::endl;
                    // 前方チェック
                    //int rcx= get_map.check_obstacle(cur_x,cur_y,drive_->_rz,0.60,0);
                    // 右横チェック
                    //int rcx= get_map.check_obstacle(cur_x,cur_y,drive_->_rz,0.60,3);
                    // 後方チェック
                    int rcx= get_map.check_obstacle(cur_x,cur_y,drive_->_rz,0.60,2);
                }
                #endif

                // GetMap::test_plot() テスト
                //#define TEST_KK_777
                #if defined(TEST_KK_777)
                {
                    drive_->get_tf(2);      // changed by nishi 2024.2.28
                    tf2::Vector3 cur_origin = drive_->base_tf.getOrigin();  // changed by nishi 2024.2.28

                    float cur_x = cur_origin.getX();    // World point(基本座標) [M]
                    float cur_y = cur_origin.getY();    // World point(基本座標) [M]
                    // ロボットの今の向き
                    std::cout << "drive->_rz*RADIANS_F: " << drive_->_rz*RADIANS_F <<std::endl;
                    get_map.test_plot(cur_x,cur_y,drive_->_rz,0.60);
                }
                #endif

                break;
            case 30:        // auto map build
                auto_map();
                break;
            case 31:        // auto map build of anchor
                auto_map_anchor();
                break;
            case 35:        // auto mower()
                auto_mower();
                break;
            case 36:        // auto mower2()
                auto_mower(2);
                break;
            case 50:
                call_service();
                std::cout << "set Navigation Mode" << std::endl;
                break;
            case 60:
                // course correct ON
                drive_cmd._course_correct = true;     // changed by nishi 2024.2.28
                std::cout << "course correct ON" << std::endl;
                break;
            case 61:
                // course correct OFF
                drive_cmd._course_correct = false;     // changed by nishi 2024.2.28
                std::cout << "course correct OFF" << std::endl;
                break;
            case 62:
                // after_correct_wait ON
                drive_cmd._after_correct_wait = true; // changed by nishi 2024.2.28
                std::cout << "after_correct_wait ON" << std::endl;
                break;
            case 63:
                // after_correct_wait OFF
                drive_cmd._after_correct_wait = false;    // changed by nishi 2024.2.28
                std::cout << "after_correct_wait OFF" << std::endl;
                break;

            case 64:
                // go curve ON
                drive_cmd._go_curve = true;   // changed by nishi 2024.2.28
                std::cout << "go curve ON" << std::endl;
                break;
            case 65:
                // go curve OFF
                drive_cmd._go_curve = false;   // changed by nishi 2024.2.28
                std::cout << "go curve OFF" << std::endl;
                break;

            case 66:    
                // set current postion to start
                std::cout << "set current postion to start" << std::endl;

                // tf map からのロボットの距離を求める。
                drive_->get_tf(2);    // tf-map z off 2022.11.15 by nishi

                cur_pos = drive_->base_tf.getOrigin();    // changed by nishi 2024.2.28

                force_start_origin=true;
                start_pos_x = cur_pos.getX();
                start_pos_y = cur_pos.getY();

                std::cout << "start_pos_x: " << start_pos_x << std::endl;
                std::cout << "start_pos_y: " << start_pos_y << std::endl;

                break;

            case 67:    // set dumper ON
                std::cout << "set dumper ON" << std::endl;
                drive_cmd._dumper=true;     // changed by nhishi 2024.2.28
                break;

            case 68:    // set dumper OFF
                std::cout << "set dumper OFF" << std::endl;
                drive_cmd._dumper=false;     // changed by nhishi 2024.2.28
                break;
            case 69:
                std::cout << "save local cost map" << std::endl;
                drive_->navi_map_save(); // changed by nhishi 2024.2.28
                break;

            case 70:    // set border top-right
                std::cout << "set border top-right" << std::endl;
                //blobFinder_.border_def.top_r.x=_goalList[goalId].x;
                //blobFinder_.border_def.top_r.y=_goalList[goalId].y;

                //anchorFinder_.border_def.top_r.x=_goalList[goalId].x;
                //anchorFinder_.border_def.top_r.y=_goalList[goalId].y;
                set_border_top_right(_goalList[goalId].x,_goalList[goalId].y);
                break;

            case 71:    // set border bottom-left
                std::cout << "set border bottom-left" << std::endl;
                //blobFinder_.border_def.bot_l.x= _goalList[goalId].x;
                //blobFinder_.border_def.bot_l.y= _goalList[goalId].y;

                //anchorFinder_.border_def.bot_l.x= _goalList[goalId].x;
                //anchorFinder_.border_def.bot_l.y= _goalList[goalId].y;
                set_border_bottom_left(_goalList[goalId].x,_goalList[goalId].y);
                break;

            case 72:    // set line_w_
                std::cout << "set line_w_" << std::endl;
                //blobFinder_.line_w_=(int)_goalList[goalId].x;
                //anchorFinder_.line_w_=(int)_goalList[goalId].x;
                set_line_w(_goalList[goalId].x);
                break;

            case 73:    // set robo_r_
                std::cout << "set robo_r_" << std::endl;
                // For Auto Map
                //blobFinder_.robo_r_=(int)_goalList[goalId].x;
                //anchorFinder_.robo_r_=(int)_goalList[goalId].x;
                set_robo_r(_goalList[goalId].x);
                break;

            case 80:    // set navigation2 mode
                std::cout << "set navigation2 mode" << std::endl;
                set_drive_mode(1);
                break;

            case 81:    // set cmd_vel mode
                std::cout << "set cmd_vel mode" << std::endl;
                set_drive_mode(0);
                break;

            case 82:    // disable error auto stop
                std::cout << "disable error auto stop" << std::endl;
                error_auto_stop_=false;
                break;

            case 99:
                f = false;
                break;
        }
        //time.sleep(1)
        goalId += 1;
    }
}

int ProControl::mloop_sub(){
    sts=0;
    int r_ct =0;

    float x,y,d_yaw,dist;

    if (t_type == 0){
        x = _goalList[goalId].x;
        y = _goalList[goalId].y;
        d_yaw = _goalList[goalId].d_yaw;
        if(force_start_origin==true){
            x += start_pos_x;
            y += start_pos_y;
        }
        if (_goalList[goalId].func == 0){
            r_ct = drive_->move_abs(x,y,d_yaw);    // changed by nishi 2024.2.28
            //self.goalMsg.pose.position.y = y;
            //self.goalMsg.pose.position.x = x;
        }
        else if (_goalList[goalId].func == 1){
            r_ct=drive_->go_abs(x,y);            // changed by nishi 2024.2.28
            //self.goalMsg.pose.position.y = y;
            //self.goalMsg.pose.position.x = x;
        }
        else if (_goalList[goalId].func == 2){
            drive_->rotate_abs(d_yaw);      // changed by nishi 2024.2.28
        }
        else if (_goalList[goalId].func == 3){
            drive_->rotate_off(d_yaw);      // changed by nishi 2024.2.28
        }
        else if(_goalList[goalId].func == 10){
            drive_->navi_move(x,y,d_yaw/RADIANS_F); // changed by nishi 2024.2.28
        }
        else if(_goalList[goalId].func == 11){
            move_abs_auto_select(x,y,d_yaw/RADIANS_F); // add by nishi 2024.4.7
        }
        else if(_goalList[goalId].func == 12){
            // ProControl::obstacle_escape(float r_lng,int black_thresh,float move_l)
            obstacle_escape(_goalList[goalId].x, (int)_goalList[goalId].y, _goalList[goalId].d_yaw);
        }
    }
    else{
        if(_goalList2[goalId].func == 0){
            dist = _goalList2[goalId].dist;
            d_yaw = _goalList2[goalId].d_yaw;
            drive_->move(dist,d_yaw);
            //self.goalMsg.pose.position.y += dist * math.sin(math.radians(d_yaw));
            //self.goalMsg.pose.position.x += dist * math.cos(math.radians(d_yaw));
        }
    }
    sleep(1);
    return r_ct;
}

#ifdef KKKKK_1
void ProControl::get_odom(){
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
void ProControl::call_service(){
    #if defined(FUTURE_USE_3)
    ros::ServiceClient client = nh_.serviceClient<std_srvs::Empty>("/rtabmap/set_mode_localization");

    std_srvs::Empty::Request req;             // リクエストの生成
    std_srvs::Empty::Response resp;           // レスポンスの生成

    bool result = client.call(req, resp);
    if(result){
        //ROS_INFO_STREAM("Recive response!");
        std::cout << "service OK" << std::endl;
    }
    else{
        //ROS_INFO_STREAM("Error!");
        std::cout << "service Error!!" << std::endl;
    }
    #endif
}


