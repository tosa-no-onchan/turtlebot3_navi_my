/*
* Programable Controller for Auto Map
*  turtlebot3_navi_my/src/pro_control_map.cpp
*
* https://docs.ros2.org/latest/api/rclcpp/classrclcpp_1_1Node.html
* https://answers.ros.org/question/307370/ros2-whats-the-best-way-to-wait-for-a-new-message/
* https://github.com/ros-planning/navigation2/blob/main/nav2_map_server/src/map_saver/map_saver.cpp
*/

#include "turtlebot3_navi_my/pro_control_map.hpp"

/*-------------------------------------
*
* class ProControlMap Members
*
--------------------------------------*/
/*
AutoMap I
auto_map()
    Auto Map builder
*/
void ProControlMap::auto_map(){
    std::cout << "Start Auto Map" << std::endl;

    float off;

    std::vector<Gpoint> *g_ponts_ptr=nullptr;

    if(blobFinder_.g_points1.empty() != true){
        blobFinder_.g_points1.clear();
    }
    if(blobFinder_.g_points2.empty() != true){
        blobFinder_.g_points2.clear();
    }
    if(blobFinder_.g_points_black.empty() != true){
        blobFinder_.g_points_black.clear();
    }
    blobFinder_.g_points1.reserve(50);
    blobFinder_.g_points2.reserve(50);
    blobFinder_.g_points_black.reserve(50);

    for(int lc=0; lc < 100 ;lc++){
        off=0.0;
        get_map.get();

        std::cout << "auto_map() #6 call drive.get_tf(2)" << std::endl;
        drive_->get_tf(2);

        tf2::Vector3 cur_origin = drive_->base_tf.getOrigin();    // changed by nishi 2024.2.28

        float cur_x = cur_origin.getX();
        float cur_y = cur_origin.getY();

        std::cout << "auto_map() #7 call blobFinder_.check()" << std::endl;
        // Find Next Unkonown Blob
        blobFinder_.check(get_map.mat_map_,get_map.mapm_,cur_x,cur_y);

        switch(blobFinder_.block_mode){
            case 0:
                g_ponts_ptr = &(blobFinder_.g_points_);
            break;
            case 1:
                g_ponts_ptr = &(blobFinder_.g_points1);
            break;
            case 2:
                g_ponts_ptr = &(blobFinder_.g_points2);
            break;
        }
        if(g_ponts_ptr->size() == 0){
            if(blobFinder_.block_mode >= 2){
                std::cout << "Auto Map next point nothing" << std::endl;
                return;
            }
            blobFinder_.block_mode++;
            std::cout << "Auto Map block_mode="<< blobFinder_.block_mode << std::endl;
            continue;
        }

        float dist, r_yaw,r_yaw_off;

        int j = g_ponts_ptr->size();
        int l=0;
        // 今回の Map 上の、blob を全て回ります。
        while(j > 0){
            j--;
            if(g_ponts_ptr->at(j).dist > 0.3){

                float ox,oy;

                // Collision Ahead - Exiting Spin spin failed が生じるの、組み込みました。将来、改善されれば、不要です。
                // ここで、ロボットの四方の障害物をチェックして、障害物から少しだけ、離れる。add by nishi 2024.3.7
                obstacle_escape();

                // 此処で、走査先の障害物との距離をチェック
                // ここの距離に余裕が必要では?  by nishi 2024.3.1
                //get_map.check_collision(g_ponts_ptr->at(j).x,g_ponts_ptr->at(j).y,ox,oy);
                get_map.check_collision(g_ponts_ptr->at(j).x,g_ponts_ptr->at(j).y,ox,oy,blobFinder_.robo_r_);   // changed by nishi 2024.3.1

                //drive.comp_dad(g_ponts_ptr->at(j).x,g_ponts_ptr->at(j).y,dist, r_yaw, r_yaw_off);
                drive_->comp_dad(ox,oy,dist, r_yaw, r_yaw_off); // changed by nishi 2024.2.28

                // Navi move
                std::cout << ">> auto_map() Next, goes to x="<< ox+off << " ,y="<< oy+off << std::endl;

                #define USE_TEST_PLOT
                #if defined(USE_TEST_PLOT)
                    // plot してみる for Debug
                    get_map.test_plot(ox+off,oy+off,r_yaw,r_yaw_off);
                #endif

                //drive.navi_move(blobFinder_.abs_x_+off,blobFinder_.abs_y_+off,r_yaw);
                //if(drive.navi_move(g_ponts_ptr->at(j).x+off,g_ponts_ptr->at(j).y+off,r_yaw,r_yaw_off)==false){
                if(drive_->navi_move(ox+off,oy+off,r_yaw,r_yaw_off)==false){    // changed by nishi 2024.2.28
                    std::cout << ">> drive_->navi_move() error end"<< std::endl;
                    std::cout << ">> black point appeend"<< std::endl;
                    blobFinder_.g_points_black.push_back(g_ponts_ptr->at(j));
                }
                std::cout << ">> Auto Map inner loop "<< l << " end"<< std::endl;
            }
            else{
                std::cout << ">> Auto Map inner loop "<< l << " pass"<< std::endl;
            }
            //末尾を削除
            g_ponts_ptr->pop_back();

            // ロボットの現在位置
            cur_origin = drive_->base_tf.getOrigin();   // changed by nishi 2024.2.28

            cur_x = cur_origin.getX();
            cur_y = cur_origin.getY();
            // ブロブを、ロボットの現在位置の近い順(降順)にする
            blobFinder_.sort_blob(cur_x,cur_y);
            j = g_ponts_ptr->size();
            l++;
        }
        if(lc==0){
            blobFinder_.block_mode=1;
        }
        g_ponts_ptr->clear();

        std::cout << ">> Auto Map >>> lc="<< lc << std::endl;

        // ここで、GetMap::check_obstacle() で、前方の障害物をチェックして、問題があれば、後ろを向かせる。add by nishi 2024.3.7
        obstacle_escape();

        // ここを、cmd_vel モードにしたらどう?  by nishi 2024.3.1
        set_drive_mode(0);      // set cmd_vel mode

        float d_yaw=90.0;
        // 360度回転
        for(int k=0;k<4 ;k++){
           drive_->rotate_off(d_yaw);
        }

        // 前方、60 をチェック
        //drive_->rotate_off(30.0);   // changed by nishi 2024.2.28
        //drive_->rotate_off(-60.0);  // changed by nishi 2024.2.28
        //drive_->rotate_off(30.0);   // changed by nishi 2024.2.28
        set_drive_mode(1);      // set nav2 mode

    }
    std::cout << ">> End Auto Map" << std::endl;
}


/*
AutoMap II
auto_map_anchor
    Auto Map builder of Anchor

*/
void ProControlMap::auto_map_anchor(){

    std::cout << "Start Auto Map Anchor" << std::endl;

    float off;

    std::vector<Gpoint> *g_ponts_ptr=nullptr;

    if(anchorFinder_.g_points1.empty() != true){
        anchorFinder_.g_points1.clear();
    }
    if(anchorFinder_.g_points2.empty() != true){
        anchorFinder_.g_points2.clear();
    }
    if(blobFinder_.g_points_black.empty() != true){
        anchorFinder_.g_points_black.clear();
    }
    anchorFinder_.g_points1.reserve(G_POINTS_MAX1_2);
    anchorFinder_.g_points2.reserve(G_POINTS_MAX1_2);
    anchorFinder_.g_points_black.reserve(G_POINTS_MAX1_2);

    // AnchorFinder::blk_ の初期化 
    anchorFinder_.init();

    // 今は、アンカー位置とロボットの近い順に走査をしている。
    // では無くて、アンカーのコリジョンチェック後の位置とロボットの近い順にすべきかでは無いか?
    for(int lc=0; lc < 100 ;lc++){
        off=0.0;

        get_map.get();

        std::cout << "auto_map_anchor() #1 call drive.get_tf(2)" << std::endl;
        drive_->get_tf(2);      // changed by nishi 2024.2.28

        //tf::Vector3 cur_origin = drive.base_tf.getOrigin();
        tf2::Vector3 cur_origin = drive_->base_tf.getOrigin();  // changed by nishi 2024.2.28

        float cur_x = cur_origin.getX();    // World point(基本座標)
        float cur_y = cur_origin.getY();    // World point(基本座標)

        std::cout << "auto_map_anchor() #2 call anchorFinder_.check()" << std::endl;
        // Find Next All Anchors
        //#if defined(USE_FUTURE_GET_MAP)
            anchorFinder_.check(&get_map,cur_x,cur_y);
        //#endif

        switch(anchorFinder_.block_mode){
            case 0:
                g_ponts_ptr = &(anchorFinder_.g_points_);
            break;
            case 1:
                g_ponts_ptr = &(anchorFinder_.g_points1);
            break;
            case 2:
                g_ponts_ptr = &(anchorFinder_.g_points2);
            break;
        }
        if(g_ponts_ptr->size() == 0){
            if(anchorFinder_.block_mode >= 2){
                std::cout << "Auto Map Anchor next point nothing" << std::endl;
                return;
            }
            anchorFinder_.block_mode++;
            std::cout << "Auto Map Anchor block_mode="<< anchorFinder_.block_mode << std::endl;
            continue;
        }

        float dist, r_yaw,r_yaw_off;

        int j = g_ponts_ptr->size();
        int l=0;
        // 今回の Map 上の、anchors を全て回ります。
        while(j > 0){
            j--;
            u_int8_t mark=255;
            if(g_ponts_ptr->at(j).dist > 0.25){

                // Collision Ahead - Exiting Spin spin failed が生じるの、組み込みました。将来、改善されれば、不要です。
                // ここで、ロボットの四方の障害物をチェックして、障害物から少しだけ、離れる。add by nishi 2024.3.7
                obstacle_escape();

                //float ox,oy;
                // 此処で、走査先の非障害物との距離をチェック
                //get_map.check_collision(g_ponts_ptr->at(j).x,g_ponts_ptr->at(j).y,ox,oy,7,1);

                drive_->comp_dad(g_ponts_ptr->at(j).x,g_ponts_ptr->at(j).y,dist, r_yaw, r_yaw_off); // changed by nishi 2024.2.28

                // Navi move
                //drive.navi_move(blobFinder_.abs_x_+off,blobFinder_.abs_y_+off,r_yaw);
                std::cout << ">> auto_map_anchor() Next, goes to x="<< g_ponts_ptr->at(j).x+off << " ,y="<< g_ponts_ptr->at(j).y+off << std::endl;

                #define USE_TEST_PLOT2
                #if defined(USE_TEST_PLOT2)
                    // plot してみる for Debug
                    get_map.test_plot(g_ponts_ptr->at(j).x+off,g_ponts_ptr->at(j).y+off,r_yaw,r_yaw_off);
                #endif

                if(drive_->navi_move(g_ponts_ptr->at(j).x+off,g_ponts_ptr->at(j).y+off,r_yaw,r_yaw_off)==false){    // changed by nishi 2024.2.28
                    std::cout << ">> drive_->navi_move() error end"<< std::endl;
                    //std::cout << ">> black point appeend"<< std::endl;
                    //anchorFinder_.g_points_black.push_back(g_ponts_ptr->at(j));
                    mark=128;
                }
                std::cout << ">> Auto Map Anchor inner loop "<< l << " end"<< std::endl;
            }
            else{
                std::cout << ">> Auto Map Anchor inner loop "<< l << " pass"<< std::endl;
            }
            // 走査済を設定
            anchorFinder_.mark_blk(g_ponts_ptr->at(j).xi,g_ponts_ptr->at(j).yi,mark);

            //末尾を削除
            g_ponts_ptr->pop_back();

            // ロボットの現在位置
            cur_origin = drive_->base_tf.getOrigin();   // changed by nishi 2024.2.28

            cur_x = cur_origin.getX();
            cur_y = cur_origin.getY();
            // ブロブを、ロボットの現在位置の近い順(降順)にする
            anchorFinder_.sort_blob(cur_x,cur_y);
            j = g_ponts_ptr->size();
            l++;
        }
        if(lc==0){
            anchorFinder_.block_mode=1;
        }
        g_ponts_ptr->clear();

        std::cout << ">> Auto Map Anchor >>> lc="<< lc << std::endl;

        // AnchorFinder::blk_ を、画像に保存します。
        anchorFinder_.save_blk();

        float d_yaw=90.0;
        // 360度回転
        //for(int k=0;k<4 ;k++){
        //   drive.rotate_off(d_yaw);
        //}

        // ここで、GetMap::check_obstacle() で、前方の障害物をチェックして、問題があれば、後ろを向かせる。add by nishi 2024.3.7
        obstacle_escape();

        // ここを、cmd_vel モードにしたらどう?  by nishi 2024.3.1
        // 前方、60 をチェック
        set_drive_mode(0);  // set cmd_vel mode
        drive_->rotate_off(30.0);   // changed by nishi 2024.2.28
        drive_->rotate_off(-60.0);  // changed by nishi 2024.2.28
        drive_->rotate_off(30.0);   // changed by nishi 2024.2.28
        set_drive_mode(1);  // set nav2 mode

    }
    anchorFinder_.release_blk();
    std::cout << ">> End Auto Map Anchor" << std::endl;
}
