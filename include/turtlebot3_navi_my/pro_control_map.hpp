/*
* Programable Controller with Auto Map
*  turtlebot3_navi_my/include/turtlebot3_navi_my/pro_control_map.hpp
*
* pro_control_map.hpp
*
*/

#ifndef PRO_CONTROL_MAP_HPP
#define PRO_CONTROL_MAP_HPP

#include "pro_control.hpp"


/*----------------------------
- class Programable Controller for Auto Map
-  build in auto_map() and auto_map_anchor()
----------------------------*/
class ProControlMap: public ProControl
{
private:

    BlobFinder blobFinder_;
    AnchorFinder anchorFinder_;

public:

    ProControlMap(){}

    void auto_map();
    void auto_map_anchor();

    // set border top-right
    void set_border_top_right(float x, float y){
        blobFinder_.border_def.top_r.x=x;
        blobFinder_.border_def.top_r.y=y;

        anchorFinder_.border_def.top_r.x=x;
        anchorFinder_.border_def.top_r.y=y;
    }
    // set border bottom-left
    void set_border_bottom_left(float x, float y){
        blobFinder_.border_def.bot_l.x=x;
        blobFinder_.border_def.bot_l.y=y;

        anchorFinder_.border_def.bot_l.x=x;
        anchorFinder_.border_def.bot_l.y=y;
    }
    // set line_w_
    void set_line_w(float x){
        blobFinder_.line_w_=(int)x;
        anchorFinder_.line_w_=(int)x;
    }
    // set robo_r_
    void set_robo_r(float x){
        blobFinder_.robo_r_=(int)x;
        anchorFinder_.robo_r_=(int)x;
    }

};


#endif      // MULTI_GOALS_H


