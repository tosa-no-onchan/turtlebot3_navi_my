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


#include "Labeling.hpp"

/*----------------------------
- class AnchorFinder 
----------------------------*/
class AnchorFinder{
private:
    bool view_f=false;
    bool view_f_m=false;

    // blob 重心
    double x_g;
    double y_g;

    GetMap *getmap_;

    cv::Mat img_lab_;

    // Map
    //Yaml yaml_;
    MapM mapm_;

    // 25[cm] Grid
    //Yaml grid_yaml_;
    MapM grid_mapm_;
    // grid ブロック table サイズ
    int size_x_;
    int size_y_;

    // 50[cm] Block
    cv::Mat blk_;
    //Yaml blk_yaml_;
    MapM blk_mapm_;

    bool blk_f_=false;
    //int blk_size_;   // blk_ 1 block size [cm] / 1dot = 50[cm]
    //int blk_w_;      // blk_  width
    //int blk_h_;      // blk_ height

public:
    // ブロブの作成時の、1[dot]の大きさ。あまり大きいと、ブロブが出来ないので注意。
    int  line_w_ = 5;  // ラインの幅 -> grid size [dot]  0.05[m] * 5 = 25[cm]
    // 障害物との距離の調整に使います。単位: size [dot]
    int  robo_r_ = 4;  // robot 半径  -> grid size [dot]  0.05[m] * 5 = 25[cm]
                       //     waffle 281 x 306[mm]    30.6/5[cm] = 6.12 -> 7/2  -> 3.5 -> 4

    // ロボットの第一候補位置(基本座標)
    double abs_x_;
    double abs_y_;
    double yaw_;

    int point_n_;   // ラベル総数

    u_char block_mode = 0;  // 走査ブロクモード 0: 全て対象 1: ブロック1 が対象 2: ブロック2 が対象
    float block_line_x = 0.0; // ブロック分け X軸 ライン

    std::vector<Gpoint> g_points_;
    std::vector<Gpoint> g_points1;
    std::vector<Gpoint> g_points2;

    std::vector<Gpoint> g_points_black;

    int anchor_;

    //std::list<Gpoint> g_points_list;
    //std::list<Gpoint> g_points1_list;
    //std::list<Gpoint> g_points2_list;

    // ロボットの行動範囲 / ボーダー定義
    // top-left (+x,+y) / bottom-right(-x,-y) [M]
    BorderBox border_def={{10.525,10.525},        //  top-left(+x,+y)
                         {-10.525,-10.525}};      //  bottom-right(-x,-y)

    AnchorFinder(){}
    ~AnchorFinder(){
        if(blk_f_ == true){
            //delete[] blk_;
            blk_f_=false;
        }
    }
    void init();
    void release_blk(){
        if(blk_f_ == true){
            //delete[] blk_;
            blk_f_=false;
        }
    }

    void sort_blob(float cur_x,float cur_y);
    void check(GetMap *getmap,float cur_x,float cur_y);

    void anchoring(cv::Mat &mat_blob2,float cur_x,float cur_y);
    bool anchor_put(int gx,int gy,float cur_x,float cur_y);
    bool check_Border(float x,float y);
    void mark_blk_world(float wx,float wy);
    void mark_blk(int bx,int by,u_int8_t mark=255);
    //void gridToWorld(int gx, int gy, float& wx, float& wy);
    //void worldToBlock(float wx, float wy,int& bx,int& by);
    void save_blk();
};

/*----------------------------
- class BlobFinder
----------------------------*/
class BlobFinder{
private:
    bool view_f=false;
    bool view_f_m=false;

    // blob 重心
    double x_g;
    double y_g;

    cv::Mat img_lab_;

public:
    // ブロブの作成時の、1[dot]の大きさ。あまり大きいと、ブロブが出来ないので注意。
    int   line_w_ = 5;  // ラインの幅 -> grid size [dot]  0.05[m] * 5 = 25[cm]
    // 障害物との距離の調整に使います。単位: size [dot]
    int   robo_r_ = 4;  // robot 半径  -> grid size [dot]  0.05[m] * 5 = 25[cm]
                        //     waffle 281 x 306[mm]    30.6/5[cm] = 6.12 -> 7/2  -> 3.5 -> 4
    // ロボットの第一候補位置(基本座標)
    double abs_x_;
    double abs_y_;
    double yaw_;

    int point_n_;   // ラベル総数

    u_char block_mode = 0;  // 走査ブロクモード 0: 全て対象 1: ブロック1 が対象 2: ブロック2 が対象
    float block_line_x = 0.0; // ブロック分け X軸 ライン

    std::vector<Gpoint> g_points_;
    std::vector<Gpoint> g_points1;
    std::vector<Gpoint> g_points2;

    std::vector<Gpoint> g_points_black;

    //std::list<Gpoint> g_points_list;
    //std::list<Gpoint> g_points1_list;
    //std::list<Gpoint> g_points2_list;

    // ロボットの行動範囲 / ボーダー定義
    // top-left (+x,+y) / bottom-right(-x,-y) [M]
    BorderBox border_def={{10.0,10.0},        //  top-left(+x,+y)
                         {-10.0,-10.0}};      //  bottom-right(-x,-y)

    BlobFinder(){}

    void sort_blob(float cur_x,float cur_y);
    void check(cv::Mat mat_map,MapM &mapm,float cur_x,float cur_y);
    bool check_Border(float x,float y);
};

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


