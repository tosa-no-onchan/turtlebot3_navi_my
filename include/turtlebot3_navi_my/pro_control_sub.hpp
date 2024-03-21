/*
* pro_control_sub.hpp
*
*/

#ifndef PRO_CONTROL_SUB_HPP
#define PRO_CONTROL_SUB_HPP

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <string>


#include <math.h>
#include <cstdio>
#include <vector>
#include <list>
#include <algorithm>
#include <functional>

//#include "ros/console.h"
//#include "nav_msgs/GetMap.h"
//#include <tf/LinearMath/Matrix3x3.h>
//#include <geometry_msgs/Quaternion.h>


#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/transform_datatypes.h"


#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
//#include "nav_msgs/srv/get_map.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"


#include "com_def.hpp"
#include "com_lib.hpp"
#include "Labeling.hpp"
#include <opencv2/opencv.hpp>

// add for ROS2
#include <rclcpp/wait_for_message.hpp>

//#include "nav2_msgs/srv/save_map.hpp"

//#define USE_MAP_SAVER 
// map_server/map_saver の画像を使う時は、こちらを使って下さい。
#if defined(USE_MAP_SAVER)
    #define FREE_AREA 254
    #define UNKNOWN_AREA 255

// オンちゃん独自の  保管画像を使う時は、こちら
#else
    #define FREE_AREA 0xff
    #define UNKNOWN_AREA 0x80
#endif

#define COLOR_1 40

#define G_POINTS_MAX 70
#define G_POINTS_MAX1_2 50


int compare_int(const void *a, const void *b);
bool compare_Gpoint_dist_min(Gpoint &s1,Gpoint &s2);
bool compare_Gpoint_dist_max(Gpoint &s1,Gpoint &s2);
void condense_Gpoint(std::vector<Gpoint> *gp);
bool find_Gpoint(float x,float y,std::vector<Gpoint> &gp);
#ifdef USE_CHECK_COLL
void check_collision(float x,float y,float &ox,float &oy,cv::Mat& mat_bin_map,MapM& mapm,int func);
#endif
//void gridToWorld(int gx, int gy, float& wx, float& wy,Yaml& yaml);
void gridToWorld(int gx, int gy, float& wx, float& wy,MapM& mapm);
//bool worldToGrid(float wx, float wy,int& gx,int& gy,Yaml& yaml);
bool worldToGrid(float wx, float wy,int& gx,int& gy,MapM& mapm);


/*----------------------------
- class GetMap
----------------------------*/
class GetMap
{
private:
    //ros::NodeHandle nh_;
    std::shared_ptr<rclcpp::Node> node_;

    //ros::Subscriber _sub;

    int _free_thresh;

    int _line_w;
    int _car_r;
    bool _match_rviz;

    std::string map_frame_;

    //nav_msgs::MapMetaData map_info;

    float resolution;
    int free_thresh;
    double org_x,org_y;
    //int x_size,y_size; 

    //Grid grid_;

    bool init_ok=false;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscript_;

    // https://qiita.com/hmito/items/db3b14917120b285112f
    std::shared_ptr<nav_msgs::msg::OccupancyGrid> map_ptr_;
    int map_ptr_cnt_=0;
    int func_;

public:

    cv::Mat mat_map_;
    cv::Mat mat_bin_map_;   // map 障害物の　2値化
    cv::Mat mat_bin_free_map_;  // map 非障害物の 2値化
    //Yaml yaml_;
    MapM mapm_;

    GetMap(){}

    //void init(ros::NodeHandle &nh,std::string map_frame="map");
    void init(std::shared_ptr<rclcpp::Node> node, int func=0, std::string map_frame="map");
    //void init(std::shared_ptr<rclcpp::Node> node,std::string map_frame="/map");

    void topic_callback(const nav_msgs::msg::OccupancyGrid & map_msg);

 
    /*
    * https://answers.ros.org/question/293890/how-to-use-waitformessage-properly/
    * http://docs.ros.org/en/lunar/api/nav_msgs/html/msg/OccupancyGrid.html
    */
    void get(bool save_f=false);

    /*
    * conv_fmt2(nav_msgs::OccupancyGrid_ map_msg)
    */
    //void conv_fmt2(boost::shared_ptr<const nav_msgs::OccupancyGrid_<std::allocator<void>>> map);
    void conv_fmt2(std::shared_ptr<const nav_msgs::msg::OccupancyGrid> map);

    //void saveMap(boost::shared_ptr<const nav_msgs::OccupancyGrid_<std::allocator<void>>> map);
    void saveMap(const nav_msgs::msg::OccupancyGrid &map,bool save_f=false);

    void check_collision(float x,float y,float &ox,float &oy,int r=5,int func=0);

    int check_obstacle(float x,float y,float rz,float r_lng,int func=0,int black_thresh=15);

    bool test_plot(float x,float y,float r_yaw,float robot_r=0.3);

    #ifdef XXXX_X
    void conv_dot2grid(self,x,y);
    /*
    convert Grid to Meter World
    */
    void conv_grid2meter(self,gx,gy,f_or_l);
    /*
    Rviz と同じ図形 にした場合。
    左上: M(+x,+y)    右上: M(-x,+y)
    左下: M(+x,-y)    右下: M(-x,-y)
    */
    void conv_grid2meter_m_p(self,gx,gy,f_or_l);
    /*
    Map データの図形 : y =  x の線対称の図形(x,y の入れ替え)
    左上: M(-x,-y)    右上: M(-x,+y)
    左下: M(+x,-y)    右下: M(+x,+y)
    */
    void conv_grid2meter_m_m(self,gx,gy,f_or_l);

    void get_rotation_matrix(self,rad);
    void np_resize(self,m_grid,req_size);
    #endif
};

/*----------------------------
- class Grid
----------------------------*/
class Grid{
private:
    std::shared_ptr<rclcpp::Node> node_;
    u_int32_t width_;
    u_int32_t height_;
    float resolution_;
    double origin_x_, origin_y_;    // map 左下 (origin_x_, origin_y_)
    //double x0_,y0_;                 // 左上 原点 (x0_,y0_)

    int line_w_;
    int size_x_;
    int size_y_;
    //boost::shared_ptr<int8_t[]> blk_;
    //std::shared_ptr<int8_t[]> blk_;
    int8_t* blk_=nullptr;

public:
    bool init_ok=false;
    Grid(){}

    //void init(nav_msgs::MapMetaData map_info,int line_w,std::vector<int8_t> data);
    void init(nav_msgs::msg::MapMetaData map_info,int line_w,std::vector<int8_t> data);
    void updateGrid(std::vector<int8_t> data);
    void saveGrid();

    void gridToWorld(unsigned int gx, unsigned int gy, double& wx, double& wy);
    bool worldToGrid(double wx, double wy, unsigned int& gx, unsigned int& gy);

    ~Grid(){
        if (blk_ != nullptr){
            delete[] blk_;
            blk_ = nullptr;
        }
    }
};



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

#endif      // GET_MAP_HPP
