/*
* Programable Controller for Auto Mower
*  turtlebot3_navi_my/src/pro_control_mower.cpp
*
*/

#include "turtlebot3_navi_my/pro_control_mower.hpp"

/*
* Contours Builder Class
*  members
*/


/*-------------------------------------
*
* class ProControlMower Members
*
--------------------------------------*/
/*
auto_mower()
  1) コース計画を作成
  2) 上記、計画にしたがって、全経路を走行する。
*/
void ProControlMower::auto_mower(){
    std::cout << "Start Auto Mower" << std::endl;

    get_map.get(true);

    drive_->get_tf(2);

    tf2::Vector3 cur_origin = drive_->base_tf.getOrigin();

    float cur_x = cur_origin.getX();
    float cur_y = cur_origin.getY();

    #define USE_TEST_PLOT
    #if defined(USE_TEST_PLOT)
        // plot してみる for Debug
        get_map.test_plot(cur_x,cur_y,drive_->_rz,drive_->_rz);
    #endif

    ContoBuilder contbuilder;
    contbuilder.init();
    // real world 座標を、Mat map 座標に変換 
    int px = (int)((cur_x - get_map.mapm_.origin[0]) / get_map.mapm_.resolution);
    int py = (int)((cur_y - get_map.mapm_.origin[1]) / get_map.mapm_.resolution);

    contbuilder.get_bolb(px,py);

}