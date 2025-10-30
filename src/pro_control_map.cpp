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
init()
*/
void ProControlMap::init(std::shared_ptr<rclcpp::Node> node){
    //ProControl::init(node);
    // changed by nishi 2024.8.31  use local_costmap
    ProControl::init(node,true);

    // ブロブの作成時の、1[dot]の大きさ。あまり大きいと、ブロブが出来ないので注意。
    int  line_w = 5;  // ラインの幅 -> grid size [dot]  0.05[m] * 5 = 25[cm]
    // 障害物との距離の調整に使います。単位: size [dot]
    int  robo_r = 4;  // robot 半径  -> grid size [dot]  0.05[m] * 5 = 25[cm]


    // init params
	node_->declare_parameter<int>("line_w", line_w);
	node_->declare_parameter<int>("robo_r", robo_r);
    node_->declare_parameter<float>("r_lng", r_lng_);
	node_->declare_parameter<int>("black_thresh", black_thresh_);
	node_->declare_parameter<float>("move_l", move_l_);

    // get params
	node_->get_parameter<int>("line_w", line_w);
	node_->get_parameter<int>("robo_r", robo_r);
    node_->get_parameter<float>("r_lng",r_lng_);  // add by nishi 2024.9.21
    node_->get_parameter<int>("black_thresh",black_thresh_);    // add by nishi 2025.10.27
    node_->get_parameter<float>("move_l",move_l_);  // add by nishi 2024.9.21

    blobFinder_.line_w_=line_w;
    anchorFinder_.line_w_=line_w;

    blobFinder_.robo_r_=robo_r;
    anchorFinder_.robo_r_=robo_r;

}
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

        std::cout << "auto_map() #7.1 blobFinder_.block_mode:"<< +blobFinder_.block_mode << std::endl;

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
            std::cout << "Auto Map next point nothing" << std::endl;
            return;
        }

        float dist, r_yaw,r_yaw_off;

        int l=0;
        // add by nishi 2025.10.26
        bool sw_end=false;
        // 今回の Map 上の、blob を全て回ります。
        // 先頭が一番大きいブロブ
        for(int j=0; j < g_ponts_ptr->size(); j++){
            if(g_ponts_ptr->at(j).dist > 0.3){
            // 1.5[m] 以上にしてみる changed by nishi 2025.10 26
            //if(g_ponts_ptr->at(j).dist > 1.5){

                float ox,oy;

                // Collision Ahead - Exiting Spin spin failed が生じるの、組み込みました。将来、改善されれば、不要です。
                // ここで、ロボットの四方の障害物をチェックして、障害物から少しだけ、離れる。add by nishi 2024.3.7
                obstacle_escape(r_lng_,black_thresh_,move_l_);

                std::cout <<"g_ponts_ptr->at(j).x="<< g_ponts_ptr->at(j).x << " , g_ponts_ptr->at(j).y=" << g_ponts_ptr->at(j).y << std::endl;

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
                    //cv::waitKey(0);
                #endif

                //drive.navi_move(blobFinder_.abs_x_+off,blobFinder_.abs_y_+off,r_yaw);
                //if(drive.navi_move(g_ponts_ptr->at(j).x+off,g_ponts_ptr->at(j).y+off,r_yaw,r_yaw_off)==false){
                if(drive_->navi_move(ox+off,oy+off,r_yaw,r_yaw_off)==false){    // changed by nishi 2024.2.28
                    std::cout << ">> drive_->navi_move() error end"<< std::endl;
                    std::cout << ">> black point append"<< std::endl;
                    blobFinder_.g_points_black.push_back(g_ponts_ptr->at(j));   // ブラックポイントリストへ入れる
                }
                std::cout << ">> Auto Map inner loop "<< l << " end"<< std::endl;
                // 試しに、1回でやめてみる add by nishi 2025.10.26
                sw_end=true;
            }
            else{
                std::cout << ">> Auto Map inner loop "<< l << " pass"<< std::endl;
            }
            // Vector は、末尾しか削除できない。
            //末尾を削除
            //g_ponts_ptr->pop_back();

            // ロボットの現在位置
            //cur_origin = drive_->base_tf.getOrigin();   // changed by nishi 2024.2.28

            //cur_x = cur_origin.getX();
            //cur_y = cur_origin.getY();
            // ブロブを、ロボットの現在位置の近い順(降順)にする
            //blobFinder_.sort_blob(cur_x,cur_y);
            l++;
            // 1回で終了にしてみる by nishi 2025.10.26
            if(sw_end == true){
                break;
            }
        }
        g_ponts_ptr->clear();

        std::cout << ">> Auto Map >>> lc="<< lc << " end!"<<std::endl;

        // ここで、GetMap::check_obstacle() で、前方の障害物をチェックして、問題があれば、後ろを向かせる。add by nishi 2024.3.7
        //obstacle_escape(r_lng_,black_thresh_,move_l_);

        //#define USE_TURN_AROUND
        // 動きが多いので、取ってみる。
        #if defined(USE_TURN_AROUND)
            // ここを、cmd_vel モードにしたらどう?  by nishi 2024.3.1
            set_drive_mode(0);      // set cmd_vel mode

            std::cout << ">> Auto Map Turn around 360 start"<<std::endl;
            float d_yaw=90.0;
            // 360度回転
            for(int k=0;k<4 ;k++){
            drive_->rotate_off(d_yaw);
            }
            std::cout << ">> Auto Map Turn around 360 stop"<<std::endl;

            // 前方、60 をチェック
            //drive_->rotate_off(30.0);   // changed by nishi 2024.2.28
            //drive_->rotate_off(-60.0);  // changed by nishi 2024.2.28
            //drive_->rotate_off(30.0);   // changed by nishi 2024.2.28
            set_drive_mode(1);      // set nav2 mode
        #endif

    }
    std::cout << ">> End Auto Map" << std::endl;
}

/*
AutoMap I
auto_map()
    Auto Map builder
*/
void ProControlMap::auto_map_old(){
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
            //if(g_ponts_ptr->at(j).dist > 0.3){
            // 1.5[m] 以上にしてみる changed by nishi 2025.10 26
            if(g_ponts_ptr->at(j).dist > 1.5){

                float ox,oy;

                // Collision Ahead - Exiting Spin spin failed が生じるの、組み込みました。将来、改善されれば、不要です。
                // ここで、ロボットの四方の障害物をチェックして、障害物から少しだけ、離れる。add by nishi 2024.3.7
                obstacle_escape(r_lng_,black_thresh_,move_l_);

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
        obstacle_escape(r_lng_,black_thresh_,move_l_);

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
                obstacle_escape(r_lng_,black_thresh_,move_l_);

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
        obstacle_escape(r_lng_,black_thresh_,move_l_);

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

/*-------------------------
* class AnchorFinder
* init()
* blk_ の初期化をします。
* 1blk : 50[cm] x 50[cm]  -> 1[dot] = 50[cm]
--------------------------*/
void AnchorFinder::init(){
    blk_mapm_.resolution = 0.25;         // 1dot = 0.25[M]
    blk_mapm_.origin[0] = border_def.bot_l.x;        // x = -10.525
    blk_mapm_.origin[1] = border_def.bot_l.y;        // y = -10.525
    blk_mapm_.origin[2] = 0.0;                      // yaw
    int blk_size = int(blk_mapm_.resolution*100.0);               // 1[dot] = 25[cm]
    float w = (border_def.top_r.x + std::fabs(border_def.bot_l.x)) / blk_mapm_.resolution;    // 42.1
    float h = (border_def.top_r.y + std::fabs(border_def.bot_l.y)) / blk_mapm_.resolution;    // 42.1
    //float ceil(float x); 
    //blk_w_ = (int)ceil(w);    // 85[dot]
    //blk_h_ = (int)ceil(h);    // 85[dot]
    blk_mapm_.width = (int)ceil(w);      // 85[dot]
    blk_mapm_.height = (int)ceil(h);      // 85[dot]

    //std::cout << "blk_h_=" << blk_h_ << std::endl;
    //std::cout << "blk_w_=" << blk_w_ << std::endl;

    //blk_ = cv::Mat::zeros(y,x,CV_8U);
    blk_ = cv::Mat::zeros(blk_mapm_.height,blk_mapm_.width,CV_8U);     //  85 x 85
    blk_f_=true;

    std::cout << "AnchorFinder::init() img_width="  << blk_mapm_.width << " ,img_height=" << blk_mapm_.height << std::endl;

    //std::exit(0);
}
/*-------------------------
* class AnchorFinder
* check_Border()
--------------------------*/
bool AnchorFinder::check_Border(float x,float y){
    if(x > border_def.top_r.x || x < border_def.bot_l.x)
        return false;
    if(y > border_def.top_r.y || y < border_def.bot_l.y)
        return false;
    return true;
}
/*-------------------------
* class AnchorFinder
* mark_blk_world(float x,float y)
* float wx: 基本座標 X
* float wy: 基本座標 Y
* blk_ に、使用済をセット
--------------------------*/
void AnchorFinder::mark_blk_world(float wx,float wy){
    int bx,by;

    //worldToBlock(wx, wy,bx,by);
    worldToGrid(wx, wy, bx, by,blk_mapm_);

    int blk_w_;      // blk_  width
    int blk_h_;      // blk_ height

    if(blk_w_ == 0 || blk_h_==0){
        std::cout << "mark_blk_world() error #1 blk_w_="  << blk_w_ << " ,blk_h_=" << blk_h_ << std::endl;
        //std::exit(0);
    }
    if(bx > blk_w_ || by > blk_h_){
        std::cout << "mark_blk_world() error #2 wx="  << wx << " ,wy=" << wy << std::endl;
        std::cout << " blk_w_="  << blk_w_ << " ,blk_h_=" << blk_h_ << std::endl;
        std::cout << " bx="  << bx << " ,by=" << by << std::endl;
        //while(1){ }
    }
    else{
        blk_.data[by * blk_w_ +bx]=255;
    }
}
/*-------------------------
* class AnchorFinder
* mark_blk(int bx,int by)
* int bx: Block X
* int by: Block Y
* blk_ に、使用済をセット
--------------------------*/
void AnchorFinder::mark_blk(int bx,int by,u_int8_t mark){
    if(blk_mapm_.width == 0 || blk_mapm_.height == 0){
        std::cout << "mark_blk() error #1 blk_mapm_.width="  << blk_mapm_.width << " ,blk_mapm_.height=" << blk_mapm_.height << std::endl;
        std::cout << "call  std::exit(0)" << std::endl;
        std::exit(0);
    }

    if(bx > blk_mapm_.width || by > blk_mapm_.height){
        std::cout << "mark_blk_world() error #2" << std::endl;
        std::cout << " blk_mapm_.width="  << blk_mapm_.width << " ,blk_h_=" << blk_mapm_.height << std::endl;
        std::cout << " bx="  << bx << " ,by=" << by << std::endl;
    }
    else{
        blk_.data[by * blk_mapm_.width +bx]=mark;
    }
}

/*-------------------------
* class AnchorFinder
* sort_blob()
--------------------------*/
void AnchorFinder::sort_blob(float cur_x,float cur_y){

    std::vector<Gpoint> *g_ponts_ptr=nullptr;
    //std::list<Gpoint> *g_ponts_ptr=nullptr;

    switch(block_mode){
        case 0:
            g_ponts_ptr = &g_points_;
        break;
        case 1:
            g_ponts_ptr = &g_points1;
        break;
        case 2:
            g_ponts_ptr = &g_points2;
        break;
    }
    for(int i=0;i < g_ponts_ptr->size();i++){
        float off_x = g_ponts_ptr->at(i).x - cur_x;
        float off_y = g_ponts_ptr->at(i).y - cur_y;

        g_ponts_ptr->at(i).dist = std::sqrt(off_x*off_x+off_y*off_y);

    }
    // dist でソート 大きい順
    switch(block_mode){
        case 0:
            std::sort(g_points_.begin(),g_points_.end(),compare_Gpoint_dist_max);
        break;
        case 1:
            std::sort(g_points1.begin(),g_points1.end(),compare_Gpoint_dist_max);
        break;
        case 2:
            std::sort(g_points2.begin(),g_points2.end(),compare_Gpoint_dist_max);
        break;
    }
}

/*-------------------------
* class AnchorFinder
* check()
* GetMap* getmap
* float cur_x : ロボットの位置 World point X (基本座標)
* float cur_y : ロボットの位置 World point Y (基本座標)
--------------------------*/
void AnchorFinder::check(GetMap* getmap,float cur_x,float cur_y){

    cv::Mat bin_img;
    //cv::namedWindow("gry", cv::WINDOW_NORMAL);
    //cv::namedWindow("thres", cv::WINDOW_NORMAL);
    //cv::namedWindow("reverse", cv::WINDOW_NORMAL);

    getmap_=getmap;

    //mapm_=mapm;             // map Yaml
    mapm_=getmap->mapm_;             // map Yaml

    //grid_mapm_=mapm;       // Grid Yaml
    grid_mapm_=getmap->mapm_;       // Grid Yaml
    grid_mapm_.resolution=mapm_.resolution * (float)line_w_;     // 0.25  1[dot] -> 0.25[M]

    if(view_f_m){
        //cv::namedWindow("img", cv::WINDOW_AUTOSIZE);
        cv::namedWindow("img", cv::WINDOW_NORMAL);
        cv::namedWindow("blob", cv::WINDOW_NORMAL);

    }
    if(view_f){
        //cv::namedWindow("thres", cv::WINDOW_AUTOSIZE);
        //cv::namedWindow("thres", cv::WINDOW_NORMAL);
        cv::namedWindow("unknown", cv::WINDOW_NORMAL);
        cv::namedWindow("block", cv::WINDOW_NORMAL);
        //cv::namedWindow("label", cv::WINDOW_NORMAL);
    }

    //rgb = cv::imread("aaa.png");
    //cv::cvtColor(rgb, gry, cv::COLOR_BGR2GRAY);
    //gry = cv::imread("/home/nishi/map_builder.pgm",0);
    //gry = cv::imread("../map_builder-house.pgm",0);
    //gry = cv::imread("../map_builder-gass.pgm",0);
    //gry = cv::imread("../Grid_save.pgm",0);


    //cv::imshow("gry", gry);


    //cv::waitKey(0);
    //return 0;

    //2. 障害物を、2値画像 and 反転します。
	//int thresh = 10;        // 障害物の値 0-100 / 未知領域:128  / 自由領域:255
	//threshold(gry, img_dst, thresh, 255, cv::THRESH_BINARY);
    // 障害物 < 10 だけを、白にします。
	//cv::threshold(mat_map, bin_img, thresh, 255, cv::THRESH_BINARY_INV);
    // soft copy
    bin_img=getmap->mat_bin_map_;

    //cv::imshow("BinReverse", bin_img);

    //--------------------
    // 3. ここから、上記、2値画像 を、ロボットの回転円の直径の Grid に分割します。
    // 障害物のセルが1個でもあれば、その Grid を障害物とします。
    //--------------------
    
    //int   line_w_ = 5;  // ラインの幅 -> grid size [dot]  0.05[m] * 5 = 25[cm]
    //int   line_w_ = 4;  // ラインの幅 -> grid size [dot]  0.05[m] * 4 = 20[cm]

    int8_t d;
    int x_cur;
    int y_cur;
    int d_cur;

    u_int32_t height_ = bin_img.rows;       // 421[dot]
    u_int32_t width_ = bin_img.cols;        // 421[dot]

    size_x_ = width_ / line_w_;
    size_y_ = height_ / line_w_;

    if(width_ % line_w_)
        size_x_++;
    if(height_ % line_w_)     
        size_y_++;

    cv::Mat block = cv::Mat::zeros(size_y_,size_x_,CV_8U);
    grid_mapm_.width = size_x_;
    grid_mapm_.height = size_y_;

    // all raws
    for(int y=0;y < size_y_; y++){
        // all columuns of a raw 
        for(int x=0;x < size_x_; x++){
            d=0;
            //di=0;
            //cnt_i=0;
            // set up 1 grid
            for(int yy = 0; yy < line_w_ && d==0; yy++){
                for(int xx=0; xx < line_w_ && d==0; xx++){
                    y_cur = y*line_w_ + yy;
                    x_cur = x*line_w_ + xx;
                    if(y_cur < height_&& x_cur < width_){
                        // 障害物(白セル)です?
                        if(bin_img.data[y_cur * width_ + x_cur] == 0xff){
                            d++;
                        }
                    }
                }
            }
            // 半分は、障害物(白セル)です。
            if(d >= line_w_*line_w_/13){
                block.data[y*size_x_ + x] = 0xff;
            }
        }
    }

    //cv::imshow("block", block);

    //----------------------
    // 4. 上でマークされたブロックをブロブ分割(ラベリング)します。
    // 今回は、Labeling.h を使います。
    // https://imura-lab.org/products/labeling/
    //-----------------------

    LabelingBS	labeling;

    int nlabel =0;  // 使用済ラベルの数(次割当ラベル番号)
    //int w = img.cols;
    int w = size_x_;
    //int h = img.rows;
    int h = size_y_;

    //cv::Mat mat_label = cv::Mat::zeros(h,w,CV_8U);
    //cv::Mat mat_label2 = cv::Mat::zeros(h,w,CV_8U);

	//cv::Mat mat_blob;

	cv::Mat img_lab(h,w, CV_16SC1);

    //short *result = new short[ w * h ];

    labeling.Exec( block.data, (short *)img_lab.data, w, h, true, 0 );

    point_n_ = labeling.GetNumOfResultRegions();   // ラベル総数

    std::cout << "point_n_=" << point_n_ << std::endl;

    //------------------------
    // 5. アンカーを全て求めてみます。
    //-----------------------
    Gpoint g_point;
    if(g_points_.empty() != true){
        g_points_.clear();
    }
    g_points_.reserve(G_POINTS_MAX);


    RegionInfoBS	*ri;
    float x_g,  y_g;

    std::cout <<"display all g center start" << std::endl;

    cv::Mat mat_blob2;

    anchor_=0;
 
    for(int i=0;i<point_n_;i++){
        ri = labeling.GetResultRegionInfo( i );
        ri->GetCenter(x_g,y_g);   // 重心を得る

        std::cout <<"x_g="<< x_g << " , y_g=" << y_g << std::endl;

    	cv::compare(img_lab, i+1, mat_blob2, cv::CMP_EQ); // ラベル番号1 を抽出

        // 該当ブロブの周囲アンカーを得る
        anchoring(mat_blob2,cur_x,cur_y);
    }

    if(block_mode==0){
        // g_points_ が当初予定件数を超えています。
        if(g_points_.size() >= G_POINTS_MAX){
            std::cout <<"g_points_ execced G_POINTS_MAX size()=" << g_points_.size() << std::endl;
            std::cout <<"take condense g_points_" << std::endl;
            // g_points1 をコンデンスします。
            condense_Gpoint(&g_points_);
        }
        // dist でソート 大きい順
        std::sort(g_points_.begin(),g_points_.end(),compare_Gpoint_dist_max);
    }
    else{
        //std::vector<Gpoint> pg_wk1;
        //std::vector<Gpoint> pg_wk2;

        for(int i = 0;i < g_points_.size();i++){
            Gpoint gp = g_points_[i];
            // ボーダーチェック
            if(check_Border(gp.x,gp.y)!=true){
                std::cout <<"border check NG"<< std::endl;
                continue;
            }
            // ブラックポイントチェック
            if(find_Gpoint(gp.x,gp.y,g_points_black)==true){
                std::cout <<"g_points_black fined"<< std::endl;
                continue;
            }
            // block1 へ割り振り
            if(gp.x < block_line_x){
                std::cout <<"g_points1.push_back(g_points_["<<i<<"]) "<< std::endl;
                g_points1.push_back(gp);
            }
            // block2 へ割り振り
            else{
                std::cout <<"check g_points2 same x,y "<< std::endl;
                if(find_Gpoint(gp.x,gp.y,g_points2) == false){
                    std::cout <<"g_points2.push_back(g_points_["<<i<<"]) "<< std::endl;
                    g_points2.push_back(gp);
                }
            }
        }
        if(block_mode==1){
            std::cout << "sort(g_points1)" << std::endl;
            std::cout << "g_points1.size()=" << g_points1.size() << std::endl;
            // g_points1 が当初予定件数を超えています。
            if(g_points1.size() >= G_POINTS_MAX1_2){
                std::cout <<"g_points1.size() execced G_POINTS_MAX1_2 size=" << g_points1.size() << std::endl;
                std::cout <<"take condense g_points1" << std::endl;
                // g_points1 をコンデンスします。
                condense_Gpoint(&g_points1);
            }
            // dist でソート 大きい順
            std::sort(g_points1.begin(),g_points1.end(),compare_Gpoint_dist_max);

            std::cout << "end sort(g_points1)" << std::endl;
        }
        else{
            std::cout <<"sort(g_points2)"<< std::endl;
            std::cout <<"g_points2.size()="<< g_points2.size() << std::endl;
            // g_points2 が当初予定件数を超えています。
            if(g_points2.size() >= G_POINTS_MAX1_2){
                std::cout <<"g_points2.size() execced G_POINTS_MAX1_2 size=" << g_points2.size() << std::endl;
                std::cout <<"take condense g_points2" << std::endl;
                // g_points2 をコンデンスします。
                condense_Gpoint(&g_points2);
            }
            std::sort(g_points2.begin(),g_points2.end(),compare_Gpoint_dist_max);

            std::cout << "end sort(g_points2)" << std::endl;
        }
    }
}


/*-------------------------
* class AnchorFinder
* anchoring()
* float cur_x : ロボットの位置 World point X (基本座標)
* float cur_y : ロボットの位置 World point Y (基本座標)
--------------------------*/
void AnchorFinder::anchoring(cv::Mat &mat_blob2,float cur_x,float cur_y){

    //------------------------
    // 6. 対象ブロブについて、その外周上に、一定間隔でアンカーを置きます。
    //-----------------------

    //cv::Mat mat_anchor=mat_blob2.clone();

    // 『OpenCVによる画像処理入門』(講談社) P157 周囲長 を求める
    // 始点の探索
    int height=mat_blob2.rows;
    int width=mat_blob2.cols;
    int ini_x,ini_y;
    bool ok_f=false;
    for(int y=0; y<height && ok_f==false;y++){
        for(int x=0; x < width;x++){
            if(mat_blob2.data[y*width + x]==255){
                ini_y=y;
                ini_x=x;
                ok_f=true;
                break;
            }
        }
    }
    // 4近傍 下右上左
    int rot_x[4] = {0, 1, 0, -1};
    int rot_y[4] = {1, 0, -1, 0};
    int rot=0; // 探索方向
    int perrimeter = 0; // 周囲長
    int now_x,now_y;

    int pre_x=ini_x;
    int pre_y=ini_y;


    // 印を付ける  -> グレーの X 印
    //mat_anchor.data[((int)pre_y-1)*width+(int)pre_x-1] = 128;
    //mat_anchor.data[((int)pre_y-1)*width+(int)pre_x+1] = 128;
    //mat_anchor.data[(int)pre_y*width+(int)pre_x] = 128;
    //mat_anchor.data[((int)pre_y+1)*width+(int)pre_x-1] = 128;
    //mat_anchor.data[((int)pre_y+1)*width+(int)pre_x+1] = 128;

    // 今の場所を アンカーとして、g_points_ と blk_ へ登録する。
    anchor_put(pre_x, pre_y, cur_x, cur_y);

    while(1){
        for(int i=0; i<4;i++){
            now_x = pre_x + rot_x[(rot+i)%4];
            now_y = pre_y + rot_y[(rot+i)%4];
            if(now_x < 0 || now_x > width-1 || now_y < 0 || now_y > height-1 ){
                continue;
            }
            if(mat_blob2.data[now_y * width + now_x] == 255){
                pre_x = now_x;
                pre_y = now_y;
                perrimeter++;
                // 8 Grid 毎に 処理
                //if((perrimeter%10)==0){
                if((perrimeter%8)==0){
                    // 今の場所を アンカーとして、g_points_ と blk_ へ登録する。
                    anchor_put(pre_x, pre_y, cur_x, cur_y);
                }
                rot += i+3;
                break;
            }
        }
        if(pre_x == ini_x && pre_y == ini_y){
            break;
        }
    }
    //cv::imshow("anchor", mat_anchor);
}

/*-------------------------
* class AnchorFinder
* anchor_put()
*  int gx: grid x
*  int gy: grid y
*  float cur_x : ロボットの位置 World point X (基本座標)
*  float cur_y : ロボットの位置 World point Y (基本座標)
--------------------------*/
bool AnchorFinder::anchor_put(int gx,int gy,float cur_x,float cur_y){
    Gpoint g_point;
    float wx0,wy0,wx,wy;
    float off_x,off_y;

    // Grid to World(wx0,wy0)
    gridToWorld(gx, gy, wx0, wy0,grid_mapm_);

    // wx0,wy0 の 障害物チェック
    // 非障害物へ近づく補正
    getmap_->check_collision(wx0,wy0,wx,wy,robo_r_,1);  // changed by nishi 2024.3.1

    // アンカーの場所が、未処理のブロックかチェック
    int bx,by;
    // World to Block
    if(worldToGrid(wx, wy, bx, by,blk_mapm_)==false){
        std::cout <<"anchor is out of range" << std::endl;
        return false;
    }
    if(blk_.data[by*blk_mapm_.width + bx] == 0){

        // Block アドレスをセット
        g_point.xi=bx;
        g_point.yi=by;

        // World アドレスをセット
        g_point.x = round_my<float>(wx,2);
        g_point.y = round_my<float>(wy,2);

        off_x = wx - cur_x;     // 基本座標 の ロボットとの距離 dX
        off_y = wy - cur_y;     // 基本座標 の ロボットとの距離 dY

        // Distance をセット
        g_point.dist = std::sqrt(off_x*off_x+off_y*off_y);

        anchor_++;
        std::cout <<"anchor_="<< anchor_ << std::endl;
        g_points_.push_back(g_point);
    }
    return true;
}

/*-------------------------
* class AnchorFinder
* save_blk()
* blk_ を、画像で保存します。
--------------------------*/
void AnchorFinder::save_blk(){
    cv::imwrite("/home/nishi/AnchorFinder_blk_.pgm", blk_);
}

/*-------------------------
* class BlobFinder
* find_Gpoint()
--------------------------*/
bool BlobFinder::check_Border(float x,float y){
    if(x > border_def.top_r.x || x < border_def.bot_l.x)
        return false;
    if(y > border_def.top_r.y || y < border_def.bot_l.y)
        return false;
    return true;
}
/*-------------------------
* class BlobFinder
* sort_blob()
--------------------------*/
void BlobFinder::sort_blob(float cur_x,float cur_y){

    std::vector<Gpoint> *g_ponts_ptr=nullptr;
    //std::list<Gpoint> *g_ponts_ptr=nullptr;

    switch(block_mode){
        case 0:
            g_ponts_ptr = &g_points_;
        break;
        case 1:
            g_ponts_ptr = &g_points1;
        break;
        case 2:
            g_ponts_ptr = &g_points2;
        break;
    }
    for(int i=0;i < g_ponts_ptr->size();i++){
        float off_x = g_ponts_ptr->at(i).x - cur_x;
        float off_y = g_ponts_ptr->at(i).y - cur_y;

        g_ponts_ptr->at(i).dist = std::sqrt(off_x*off_x+off_y*off_y);

    }
    // dist でソート 大きい順
    switch(block_mode){
        case 0:
            std::sort(g_points_.begin(),g_points_.end(),compare_Gpoint_dist_max);
        break;
        case 1:
            std::sort(g_points1.begin(),g_points1.end(),compare_Gpoint_dist_max);
        break;
        case 2:
            std::sort(g_points2.begin(),g_points2.end(),compare_Gpoint_dist_max);
        break;
    }
}

/*-------------------------
* class BlobFinder
* check()
* block_mode:
* reffer from : ~/usr/local/src/cpp-nishi/opencv-test1/main-9.cpp
--------------------------*/
void BlobFinder::check(cv::Mat mat_map,MapM &mapm,float cur_x,float cur_y){
    cv::Mat rgb, gry, thres,reverse,neg,dst2;
    //cv::namedWindow("gry", cv::WINDOW_NORMAL);
    //cv::namedWindow("thres", cv::WINDOW_NORMAL);
    //cv::namedWindow("reverse", cv::WINDOW_NORMAL);

    std::string img_s="1.img";
    std::string unknown_s="2.unknown";
    std::string block_s="3.block";
    std::string label_s="4.label";
    std::string blob_s="5.blob";

    if(view_f_m){
        //cv::namedWindow(img_s, cv::WINDOW_AUTOSIZE);
        cv::namedWindow(img_s, cv::WINDOW_NORMAL);
        cv::namedWindow(blob_s, cv::WINDOW_NORMAL);

    }
    if(view_f){
        //cv::namedWindow("thres", cv::WINDOW_AUTOSIZE);
        //cv::namedWindow("thres", cv::WINDOW_NORMAL);
        cv::namedWindow(unknown_s, cv::WINDOW_NORMAL);
        cv::namedWindow(block_s, cv::WINDOW_NORMAL);
        //cv::namedWindow(label_s, cv::WINDOW_NORMAL);
    }

    //rgb = cv::imread("aaa.png");
    //cv::cvtColor(rgb, gry, cv::COLOR_BGR2GRAY);
    //gry = cv::imread("/home/nishi/map_builder.pgm",0);
    //gry = cv::imread("../map_builder-house.pgm",0);
    //gry = cv::imread("../map_builder-gass.pgm",0);
    //gry = cv::imread("../Grid_save.pgm",0);

    //cv::imshow("gry", gry);

    //cv::waitKey(0);
    //return 0;

    int thresh=100;
    //int thresh=130;
    //int thresh=250;

    //cv::threshold(gry,thres,thresh,255,cv::THRESH_BINARY);

    //cv::imshow("thres", thres);

    // 白黒反転
    //thres.convertTo(reverse,thres.type(),-1.0,255.0);
    //cv::imshow("reverse", reverse);

    cv::Mat img=mat_map;
    
    //cv::Mat img = (cv::Mat_<u_int8_t>(6,6) << 
    //            0, 128, 0, 128, 128, 0,
    //            0, 255, 0, 255, 255, 0,
    //            128, 255, 255, 255, 0, 0,
    //            128, 255, 0, 0, 0, 0, 
    //            0,255, 0, 255, 255, 0,
    //            0, 0, 0, 128, 128, 0
    //            );  

    std::cout << "img.rows=" << img.rows << std::endl;
    std::cout << "img.cols=" << img.cols << std::endl;
    std::cout << "img.type()=" << img.type() << std::endl;
    std::cout << "img.step=" << img.step << std::endl;

    if(view_f_m){
        cv::imshow(img_s, img);
        //cv::waitKey(0);
        //return;
    }

    //cv::Mat dst = cv::Mat::zeros(img.rows,img.cols,CV_16U);
    cv::Mat unknown = cv::Mat::zeros(img.rows,img.cols,CV_8U);

    std::cout << "unknown.rows=" << unknown.rows << std::endl;
    std::cout << "unknown.cols=" << unknown.cols << std::endl;
    std::cout << "unknown.channels()=" << unknown.channels() << std::endl;
    std::cout << "unknown.type()=" << unknown.type() << std::endl;
    std::cout << "unknown.step=" << unknown.step << std::endl;

    //int thresh=100;
    //int thresh=130;
    //int thresh=250;

    //cv::threshold(img,thres,thresh,255,cv::THRESH_BINARY);

    //cv::imshow("thres", thres);

    //std::cout << (int)thres.at<u_int8_t>(0,0) << "," << (int)thres.at<u_int8_t>(0,1) << "," << (int)thres.at<u_int8_t>(0,2) << std::endl;
    //std::cout << (int)thres.at<u_int8_t>(1,0) << "," << (int)thres.at<u_int8_t>(1,1) << "," << (int)thres.at<u_int8_t>(1,2) << std::endl;
    //std::cout << (int)thres.at<u_int8_t>(2,0) << "," << (int)thres.at<u_int8_t>(2,1) << "," << (int)thres.at<u_int8_t>(2,2) << std::endl;

    //int nlabel =0;  // ラベルの数
    int w = img.cols;
    int h = img.rows;

    //const int TABLESIZE = 1024;
    //static int table[TABLESIZE];        // label 使用番号 管理テーブル
    //table[0]=0;

    int unknown_t[img.rows*img.cols];
    for(int y=0;y<img.rows;y++){
        for(int x=0;x < img.cols;x++){
            unknown_t[y*img.cols + x] = 0;
        }
    }
    //----------------
    // 1. 4近傍に 未知領域(-1) のある、自由領域(白) をピックアップして、unknown_t[]へ入れる
    //----------------
    for(int y = 0 ; y < h; y++){
        for (int x=0;x < w; x++){
            //std::cout << "y=" << y << ",x=" << x << std::endl;
            // 注目画素は、自由領域 でない(黒)
            if(img.data[y*w+x] != FREE_AREA){
               unknown_t[y * w + x] = 0;
            }
            // 注目画素は、自由領域(白 : 0xff) だ!!
            else{
                // 近傍8画素をチェック
                //const int N = 8;
                //const int dy[N] = { 1, 1,  1,  0, 0, -1, -1, -1};
                //const int dx[N] = {-1, 0,  1, -1, 1, -1 , 0 , 1};

                // 近傍4画素をチェック 上、下、左、右
                const int N = 4;
                const int dy[N] = { 1,  0, 0, -1};
                const int dx[N] = { 0, -1, 1,  0};

                int list[N];
                int count =0;
                for(int k = 0; k < N; k++){
                    int xdx = x + dx[k];
                    int ydy = y + dy[k];
                    int m = ydy * w + xdx;
                    // はみ出していない and  Unknown -1(0x80) セル
                    if(xdx >= 0 && ydy >= 0 && xdx < w && ydy < h && img.data[m] == UNKNOWN_AREA){
                        unknown_t[y*w+x] = 0xff;
                        unknown.data[y*w+x] = 0xff;
                        break;
                    }
                }
            }
        }
    }

    if(view_f){
        cv::imshow(unknown_s, unknown);
        //cv::waitKey(0);
        //return;
    }
    //--------------------
    // 2. ここから、上記、unknown_t を、ロボットの回転円の直径のブロックに分割します。
    // 未知領域(-1=0x80)に隣接する自由領域セル(0=0xff) が1個でもあれば、そのブロックをマーキングします。
    //--------------------

    //int   line_w_ = 5;  // ラインの幅 -> grid size [dot]  0.05[m] * 5 = 25[cm]

    int8_t d;
    int x_cur;
    int y_cur;
    int d_cur;

    u_int32_t height_ = img.rows;
    u_int32_t width_ = img.cols;

    int size_x_ = width_ / line_w_;
    int size_y_ = height_ / line_w_;

    if(width_ % line_w_)
        size_x_++;
    if(height_ % line_w_)     
        size_y_++;

    cv::Mat block = cv::Mat::zeros(size_y_,size_x_,CV_8U);
    // all raws
    for(int y=0;y < size_y_; y++){
        // all columuns of a raw 
        for(int x=0;x < size_x_; x++){
            d=0;
            //di=0;
            //cnt_i=0;
            // set up 1 grid
            for(int yy = 0; yy < line_w_ && d==0; yy++){
                for(int xx=0; xx < line_w_ && d==0; xx++){
                    y_cur = y*line_w_ + yy;
                    x_cur = x*line_w_ + xx;
                    if(y_cur < height_&& x_cur < width_){
                        // unknown セルに接している?
                        if(unknown_t[y_cur * width_ + x_cur] == 0xff){
                            d=0xff;
                        }
                    }
                }
            }
            if(d!=0){
                block.data[y*size_x_ + x] = 0xff;
            }
        }
    }

    if(view_f){
        cv::imshow(block_s, block);
        //cv::waitKey(0);
        //return;
    }

    //----------------------
    // 3. 上でマークされたブロックをブロブ分割(ラベリング)します。
    // 今回は、Labeling.h を使います。
    // https://imura-lab.org/products/labeling/
    //-----------------------
    LabelingBS	labeling;
    int nlabel =0;  // 使用済ラベルの数(次割当ラベル番号)
    //int w = img.cols;
    w = size_x_;
    //int h = img.rows;
    h = size_y_;

    //cv::Mat mat_label = cv::Mat::zeros(h,w,CV_8U);
    //cv::Mat mat_label2 = cv::Mat::zeros(h,w,CV_8U);

	cv::Mat mat_blob;
    // ブロブ分割(ラベリング) で、ブログID を埋め込んだ画像。 内容値が、blob id+1
	//cv::Mat img_lab_(h,w, CV_16SC1);
    img_lab_ = cv::Mat::zeros(h,w,CV_16SC1);

    //--------------------------------
    // labeling.Exec( src, result, w, h, true, 30 )
    //  src: 入力画像の先頭アドレス(unsigned char *)
    //  result: 出力画像の先頭アドレス(short *)
    //  w: 画像の幅(int)
    //  h: 画像の高さ(int)
    //  sort: 領域の大きさ順にソートするか(bool) – true:する false:しない
    //  min_size: 消去する小領域の最大サイズ(int) – これ以下のサイズの領域を消去する
    //--------------------------------
    labeling.Exec( block.data, (short *)img_lab_.data, w, h, true, 1 );

    point_n_ = labeling.GetNumOfResultRegions();   // ラベル総数
    std::cout << "point_n_=" << point_n_ << std::endl;

    //----------------------
    //4. ラベリングされたブロブで大きなブロブを
    // 1つ選んで、それの重心を求めて、その場所を、ロボットの移動場所とします。
    //----------------------
    RegionInfoBS	*ri;
    ri = labeling.GetResultRegionInfo( 0 );

	cv::compare(img_lab_, 1, mat_blob, cv::CMP_EQ); // ラベル番号1 を抽出

    float x_g,  y_g;
    ri->GetCenter(x_g,y_g);   // 重心を得る

	std::cout <<"w="<< w << ", h=" << h << std::endl;

    //ここが、ロボットの移動先
    // 実際使うには、 x_g,y_g を、標準座標に変換しないといけない。
	std::cout <<"x_g="<< x_g << " , y_g=" << y_g << std::endl;

    // blob の重心は、左上が、原点です
    abs_x_ = (x_g + 0.5) * (double)line_w_ * mapm.resolution + mapm.origin[0];
    abs_y_ = (y_g + 0.5) * (double)line_w_ * mapm.resolution + mapm.origin[1];
    yaw_ = mapm.origin[3];

	std::cout <<"mapm.resolution="<< mapm.resolution << std::endl;

	std::cout <<"abs_x_="<< abs_x_ << " , abs_y_=" << abs_y_ << std::endl;

    // 印を付ける  -> グレーの X 印
    mat_blob.data[((int)y_g-1)*w+(int)x_g-1] = 128;
    mat_blob.data[((int)y_g-1)*w+(int)x_g+1] = 128;
    mat_blob.data[(int)y_g*w+(int)x_g] = 128;
    mat_blob.data[((int)y_g+1)*w+(int)x_g-1] = 128;
    mat_blob.data[((int)y_g+1)*w+(int)x_g+1] = 128;
  
    if(view_f_m){
        cv::imshow(blob_s, mat_blob);
        //cv::waitKey(0);
    }

    //------------------------
    // 5. 重心を全て求めてみます。
    //-----------------------
    std::cout <<"display all g center start" << std::endl;

    Gpoint g_point;
    if(g_points_.empty() != true){
        g_points_.clear();
    }
    g_points_.reserve(point_n_);

    // ブロブの大きい順に処理する。
    for(int i=0;i < point_n_;i++){
        ri = labeling.GetResultRegionInfo( i );
        ri->GetCenter(x_g,y_g);   // ブロブの重心を得る
        std::cout <<"x_g="<< x_g << " , y_g=" << y_g << std::endl;

        // アドレスを変換する。
        float x0,y0;
        x0 = (x_g + 0.5) * (double)line_w_ * mapm.resolution + mapm.origin[0];
        y0 = (y_g + 0.5) * (double)line_w_ * mapm.resolution + mapm.origin[1];

        // ブロブの重心を、実アドレスに変換する。
        g_point.x = round_my<float>(x0,2);
        g_point.y = round_my<float>(y0,2);

        float off_x = x0 - cur_x;
        float off_y = y0 - cur_y;

        g_point.dist = std::sqrt(off_x*off_x+off_y*off_y);
        g_point.pic = ri->GetNumOfPixels();

        std::cout <<"g_point.x="<< g_point.x << " , g_point.y=" << g_point.y << std::endl;
        // ブラックポイントチェック add by nishi 2025.10.28
        if(find_Gpoint(g_point.x,g_point.y,g_points_black)==true){
            std::cout <<"g_points_black fined"<< std::endl;
            continue;
        }
        // ブロブの重心を実アドレス変換した値をリストに登録する。
        g_points_.push_back(g_point);
    }

    if(block_mode==0){
        // ブロブの大きい順にロボットの目的地にする。 changed by nishi 2025.10.29
        // dist でソート 大きい順
        //std::sort(g_points_.begin(),g_points_.end(),compare_Gpoint_dist_max);
    }
    else{
        //std::vector<Gpoint> pg_wk1;
        //std::vector<Gpoint> pg_wk2;

        for(int i = 0;i < g_points_.size();i++){
            Gpoint gp = g_points_[i];
            // ボーダーチェック
            if(check_Border(gp.x,gp.y)!=true){
                std::cout <<"border check NG"<< std::endl;
                continue;
            }
            // ブラックポイントチェック
            if(find_Gpoint(gp.x,gp.y,g_points_black)==true){
                std::cout <<"g_points_black fined"<< std::endl;
                continue;
            }
            // block1 へ割り振り
            if(gp.x < block_line_x){
                std::cout <<"g_points1.push_back(g_points_["<<i<<"]) "<< std::endl;
                g_points1.push_back(gp);
            }
            // block2 へ割り振り
            else{
                std::cout <<"check g_points2 same x,y "<< std::endl;
                if(find_Gpoint(gp.x,gp.y,g_points2) == false){
                    std::cout <<"g_points2.push_back(g_points_["<<i<<"]) "<< std::endl;
                    g_points2.push_back(gp);
                }
            }
        }

        if(block_mode==1){
            std::cout << "sort(g_points1)" << std::endl;
            std::cout << "g_points1.size()=" << g_points1.size() << std::endl;
            // dist でソート 大きい順
            //condense_Gpoint(&g_points1);
            std::sort(g_points1.begin(),g_points1.end(),compare_Gpoint_dist_max);

            std::cout << "end sort(g_points1)" << std::endl;
        }
        else{
            std::cout <<"sort(g_points2)"<< std::endl;
            std::cout <<"g_points2.size()="<< g_points2.size() << std::endl;
            //condense_Gpoint(&g_points2);
            std::sort(g_points2.begin(),g_points2.end(),compare_Gpoint_dist_max);

            std::cout << "end sort(g_points2)" << std::endl;
        }
    }
    return;
}
