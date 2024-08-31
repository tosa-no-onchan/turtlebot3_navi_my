/*
* Programable Controller for Auto Mower
*  turtlebot3_navi_my/src/pro_control_mower.cpp
*
*/

#include "turtlebot3_navi_my/pro_control_mower.hpp"

bool compare_Cource_Plan_dist_min(Cource_Plan &s1,Cource_Plan &s2){
    return s1.dist < s2.dist;
};


/*-------------------------------------
* ContoBuilderMower Class
*  members
--------------------------------------*/


/*-------------------------------------
*
* class ProControlMower Members
*
--------------------------------------*/
/*
init()
*/
void ProControlMower::init(std::shared_ptr<rclcpp::Node> node){
    //ProControl::init(node);
    // changed by nishi 2024.8.31  use local_costmap
    ProControl::init(node,true);

    node_->declare_parameter<double>("threshold",250.0);    // add by nishi 2024.4.24
    node_->declare_parameter<bool>("plann_test",false);    // add by nishi 2024.4.24
    node_->declare_parameter<bool>("all_nav2",true);    // add by nishi 2024.4.24

    node_->get_parameter<double>("threshold",threshold_);  // add by nishi 2024.4.24
    node_->get_parameter<bool>("plann_test",plann_test_);  // add by nishi 2024.4.24
    node_->get_parameter<bool>("all_nav2",all_nav2_);  // add by nishi 2024.4.24

}
/*
auto_mower(int m_type=1)
  m_type : 1 auto_mower()
           2 auto_mower2()
  1) コース計画を作成する。
  2) 上記、計画にしたがって、全経路を走行する。
*/
void ProControlMower::auto_mower(int m_type){
    std::cout << "Start Auto Mower" << std::endl;

    std::cout << " all_nav2_:" << all_nav2_ << std::endl;

    //get_map.get(true);
    if(get_map.get() != true)
    {
        std::cout << "  get_map error occured , then Auto Mower is not executable!!" << std::endl;
        return;
    }

    // test by nishi 2024.8.31
    if(get_local_map.get() != true)
    {
        std::cout << "  get_local_map error occured , then Auto Mower is not executable!!" << std::endl;
        return;
    }

    drive_->get_tf(2);
    tf2::Vector3 cur_origin = drive_->base_tf.getOrigin();

    float cur_x = cur_origin.getX();
    float cur_y = cur_origin.getY();

    // ロボットの自由領域を識別して、走行ラインの経路計画を作成します。
    //   output -> robo_slice_bolb_clst_
    ContoBuilderMower contbuilder;

    //double threshold=250;   // white 250 / gray 200

    contbuilder.init(get_map.mat_map_,get_map.mapm_,threshold_);

    // real world 座標を、Mat map 座標に変換 
    int px = (int)((cur_x - get_map.mapm_.origin[0]) / get_map.mapm_.resolution);
    int py = (int)((cur_y - get_map.mapm_.origin[1]) / get_map.mapm_.resolution);

    std::cout << " px=" << px << " py="<< py << std::endl;
    // px=87 py=94

    contbuilder.get_bolb(px,py,true);    // x,y,isCourceDisp

    #define USE_TEST_PLOT
    #if defined(USE_TEST_PLOT)
        // plot してみる for Debug
        get_map.test_plot(cur_x,cur_y,drive_->_rz);
    #endif

    #define USE_TEST_PLOT_LOCAL_MAP
    #if defined(USE_TEST_PLOT_LOCAL_MAP)
        // plot してみる for Debug
        get_local_map.test_plot(cur_x,cur_y,drive_->_rz, 0.3 ,"-local");
    #endif

    // プラン作成テストのみです。
    if(plann_test_)
        return;

    #define USE_TEST_PLOT2
    #define USE_TEST_PLOT_LOCAL_MAP2

    std::cout << " start running" << std::endl;
    // contbuilder.cource_plan_vec_ に、走行プランが作成されるので、これに沿ってロボットを走行させます。
    std::vector<Cource_Plan>::iterator cource_plan;
    for(cource_plan=contbuilder.cource_plan_vec_.begin();cource_plan != contbuilder.cource_plan_vec_.end(); cource_plan++){
        int blob_n = cource_plan->blob_n;
        std::cout << " blob_n=" << blob_n << std::endl;
        Robo_Slice_Clast robo_slice_clast = contbuilder.robo_slice_clast_vec_.at(blob_n);

        // Robo_Slice Compact を処理する
        //int num = robo_slice_clast.robo_slice_vec.size();
        //for(int num=0;num < robo_slice_clast.robo_slice_vec.size() ;num++){
        //}
        std::vector<Robo_Slice_Compat>::iterator robo_slice_compat;
        int cur_idx=0;
        for(robo_slice_compat=robo_slice_clast.robo_slice_vec.begin();robo_slice_compat!=robo_slice_clast.robo_slice_vec.end();robo_slice_compat++){
            cv::Point f = robo_slice_compat->f;
            cv::Point l = robo_slice_compat->l;

            // コースを描画してみる。
            contbuilder.cource_plot(f,l);

            float f_x,f_y,l_x,l_y;
            float yf,xf,r_yaw;

            // Mat map 座標 -> real world 座標に変換。
            // ただし、 contbuilder.init() を実行した時の情報を使うこと。
            // ratbamap localization:=true で起動しても、 Static Map がリサイズされるから。
            contbuilder.map2tr_real(f.x,f.y,f_x,f_y);
            //f_x = ((float)f.x + 0.5)  * contbuilder.mapm_.resolution + contbuilder.mapm_.origin[0];
            //f_y = ((float)f.y + 0.5)  * contbuilder.mapm_.resolution + contbuilder.mapm_.origin[1];

            contbuilder.map2tr_real(l.x,l.y,l_x,l_y);
            //l_x = ((float)l.x + 0.5)  * contbuilder.mapm_.resolution + contbuilder.mapm_.origin[0];
            //l_y = ((float)l.y + 0.5)  * contbuilder.mapm_.resolution + contbuilder.mapm_.origin[1];

            f_x = round_my<float>(f_x,2);
            f_y = round_my<float>(f_y,2);

            l_x = round_my<float>(l_x,2);
            l_y = round_my<float>(l_y,2);

            // 偶数番です。 ロボットを f へ、呼び寄せる。
            if(cur_idx%2 == 0 || m_type==2){
                // Collision Ahead - Exiting Spin spin failed が生じるの、組み込みました。将来、改善されれば、不要です。
                // ここで、ロボットの四方の障害物をチェックして、障害物から少しだけ、離れる。add by nishi 2024.3.7
                obstacle_escape();

                // 外側を向かせる
                // l -> f の向きを求める
                yf = f_y - l_y;
                xf = f_x - l_x;
                // atan2(float y, float x);
                r_yaw = std::atan2(yf,xf);

                #if defined(USE_TEST_PLOT2)
                    // target postion plot してみる for Debug
                    get_map.test_plot(f_x,f_y,r_yaw);
                #endif

                #if defined(USE_TEST_PLOT_LOCAL_MAP2)
                    // cur postion plot してみる for Debug
                    drive_->get_tf(2);
                    cur_origin = drive_->base_tf.getOrigin();
                    cur_x = cur_origin.getX();
                    cur_y = cur_origin.getY();
                    get_local_map.test_plot(cur_x,cur_y,drive_->_rz, 0.3 ,"-local");
                #endif


                if(all_nav2_ == false){
                    if(move_abs_auto_select(f_x,f_y,r_yaw)==false)    // changed by nishi 2024.2.28
                        std::cout << ">> #1 drive_->navi_move() error end"<< std::endl;
                }
                else{
                    if(drive_->navi_move(f_x,f_y,r_yaw)==false)    // changed by nishi 2024.2.28
                        std::cout << ">> #1 drive_->navi_move() error end"<< std::endl;
                }

                // Collision Ahead - Exiting Spin spin failed が生じるの、組み込みました。将来、改善されれば、不要です。
                // ここで、ロボットの四方の障害物をチェックして、障害物から少しだけ、離れる。add by nishi 2024.3.7
                obstacle_escape();

                // 外側を向かせる
                // f -> l の向きを求める
                yf = l_y - f_y;
                xf = l_x - f_x;
                // atan2(float y, float x);
                r_yaw = std::atan2(yf,xf);
                // f -> l へ向かう

                #if defined(USE_TEST_PLOT2)
                    // target postion plot してみる for Debug
                    get_map.test_plot(l_x,l_y,r_yaw);
                #endif

                #if defined(USE_TEST_PLOT_LOCAL_MAP2)
                    // cur postion plot してみる for Debug
                    drive_->get_tf(2);
                    cur_origin = drive_->base_tf.getOrigin();
                    cur_x = cur_origin.getX();
                    cur_y = cur_origin.getY();
                    get_local_map.test_plot(cur_x,cur_y,drive_->_rz, 0.3 ,"-local");
                #endif


                if(all_nav2_ == false){
                    if(move_abs_auto_select(l_x,l_y,r_yaw)==false)    // changed by nishi 2024.2.28
                        std::cout << ">> #2 drive_->navi_move() error end"<< std::endl;
                }
                else{
                    if(drive_->navi_move(l_x,l_y,r_yaw)==false)    // changed by nishi 2024.2.28
                        std::cout << ">> #2 drive_->navi_move() error end"<< std::endl;
                }
            }
            // ロボットを l へ、呼び寄せる
            else{
                // Collision Ahead - Exiting Spin spin failed が生じるの、組み込みました。将来、改善されれば、不要です。
                // ここで、ロボットの四方の障害物をチェックして、障害物から少しだけ、離れる。add by nishi 2024.3.7
                obstacle_escape();

                // 外側を向かせる
                // f -> l の向きを求める
                yf = l_y - f_y;
                xf = l_x - f_x;
                // atan2(float y, float x);
                r_yaw = std::atan2(yf,xf);

                #if defined(USE_TEST_PLOT2)
                    // target postion plot してみる for Debug
                    get_map.test_plot(l_x,l_y,r_yaw);
                #endif

                #if defined(USE_TEST_PLOT_LOCAL_MAP2)
                    // cur postion plot してみる for Debug
                    drive_->get_tf(2);
                    cur_origin = drive_->base_tf.getOrigin();
                    cur_x = cur_origin.getX();
                    cur_y = cur_origin.getY();
                    get_local_map.test_plot(cur_x,cur_y,drive_->_rz, 0.3 ,"-local");
                #endif

                if(all_nav2_ == false){
                    if(move_abs_auto_select(l_x,l_y,r_yaw)==false)    // changed by nishi 2024.2.28
                        std::cout << ">> #3 drive_->navi_move() error end"<< std::endl;
                }
                else{
                    if(drive_->navi_move(l_x,l_y,r_yaw)==false)    // changed by nishi 2024.2.28
                        std::cout << ">> #3 drive_->navi_move() error end"<< std::endl;
                }

                // Collision Ahead - Exiting Spin spin failed が生じるの、組み込みました。将来、改善されれば、不要です。
                // ここで、ロボットの四方の障害物をチェックして、障害物から少しだけ、離れる。add by nishi 2024.3.7
                obstacle_escape();

                // 外側を向かせる
                // l -> f の向きを求める
                yf = f_y - l_y;
                xf = f_x - l_x;
                // atan2(float y, float x);
                r_yaw = std::atan2(yf,xf);
                // l -> f  へ向かう

                #if defined(USE_TEST_PLOT2)
                    // target postion plot してみる for Debug
                    get_map.test_plot(f_x,f_y,r_yaw);
                #endif

                #if defined(USE_TEST_PLOT_LOCAL_MAP2)
                    // cur postion plot してみる for Debug
                    drive_->get_tf(2);
                    cur_origin = drive_->base_tf.getOrigin();
                    cur_x = cur_origin.getX();
                    cur_y = cur_origin.getY();
                    get_local_map.test_plot(cur_x,cur_y,drive_->_rz, 0.3 ,"-local");
                #endif

                if(all_nav2_ == false){
                    if(move_abs_auto_select(f_x,f_y,r_yaw)==false)    // changed by nishi 2024.2.28
                        std::cout << ">> #4 drive_->navi_move() error end"<< std::endl;
                }
                else{
                    if(drive_->navi_move(f_x,f_y,r_yaw)==false)    // changed by nishi 2024.2.28
                        std::cout << ">> #4 drive_->navi_move() error end"<< std::endl;
                }
            }
            cur_idx++;
        }
    }
}
