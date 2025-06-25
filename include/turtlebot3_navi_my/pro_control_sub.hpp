/*
* pro_control_sub.hpp
*
*/

#ifndef PRO_CONTROL_SUB_HPP
#define PRO_CONTROL_SUB_HPP

#include <rclcpp/rclcpp.hpp>
//#include "rclcpp/rclcpp.hpp"

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
// https://docs.ros2.org/foxy/api/nav_msgs/msg/OccupancyGrid.html
#include "nav_msgs/msg/occupancy_grid.hpp"
//#include "nav_msgs/srv/get_map.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

// add by nishi 2024.9.26
//#include "geometry_msgs/msg/pose.hpp"


#include "com_def.hpp"
#include "com_lib.hpp"
#include "Labeling.hpp"
#include <opencv2/opencv.hpp>

// add for ROS2
#include <rclcpp/wait_for_message.hpp>

// add by nishi 2024.4.5 for Enable topic statics
//#include "rclcpp/rclcpp.hpp"
//#include "rclcpp/subscription_options.hpp"


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
void gridToWorld(int gx, int gy, float& wx, float& wy,MapM& mapm,bool y_reverse=true);
//bool worldToGrid(float wx, float wy,int& gx,int& gy,Yaml& yaml);
bool worldToGrid(float wx, float wy,int& gx,int& gy,MapM& mapm,bool y_reverse=true);

bool tf_world2MatMap(float w_x, float w_y,int& m_x,int& m_y,MapM& mapm,bool y_reverse=true);

/*----------------------------
- class GetMap
----------------------------*/
class GetMap
{
private:
    std::shared_ptr<rclcpp::Node> node_;
    int _free_thresh;

    int _line_w;
    int _car_r;
    bool _match_rviz;

    //nav_msgs::MapMetaData map_info;

    float resolution;
    int free_thresh;
    //int x_size,y_size; 

    //Grid grid_;

    //bool init_ok=false;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscript_;

    // https://qiita.com/hmito/items/db3b14917120b285112f
    // https://docs.ros2.org/foxy/api/nav_msgs/msg/OccupancyGrid.html
    std::shared_ptr<nav_msgs::msg::OccupancyGrid> map_ptr_;
    int map_ptr_cnt_=0;
    int func_;

public:
    std::mutex mtx_;    // add by nishi 2024.9.28
    std::string map_frame_;

    cv::Mat mat_map_;
    cv::Mat mat_bin_map_;   // map 障害物の　2値化
    cv::Mat mat_bin_free_map_;  // map 非障害物の 2値化

    cv::Mat mat_map_raw_;    // mat_map_ の上下逆転 toicp の生データ add by nishi 2024.10.5
    //Yaml yaml_;
    MapM mapm_;

    double org_x,org_y;
    bool init_ok=false;     // add by nishi 2024.9.4

    bool is_static_map_=true;   // map_frmae="map" の時 true    add by nishi 2-24.9.26
    bool map_orient_fix_=true;   // costmap の時の、方角の基準 add by nishi 2024.9.26
                                // true -> global_frame: map
                                // false -> global_frame: base_footprint
    bool y_reverse_=false;      // for saveMap() mat_map_  default y order by nishi 2024.12.30
    GetMap(){}

    //void init(std::shared_ptr<rclcpp::Node> node, int func=0, std::string map_frame="map",bool is_static_map=true,bool map_orient_fix=true);
    void init(std::shared_ptr<rclcpp::Node> node, int func=0, std::string map_frame="map",bool is_static_map=true);
    //void init(std::shared_ptr<rclcpp::Node> node,std::string map_frame="/map");

    void topic_callback(const nav_msgs::msg::OccupancyGrid & map_msg);

     /*
    * get(bool save_f=false,bool costmap_f=false, bool reverse_f=true)
    *   bool save_f :  true -> save 
    *   bool inflation_f : true -> get costmap reduced inflation size
    *   bool y_reverse  : y axis reverse
    *       true -> map => cv::Mat 変換で、y軸を逆順にコピーする。 
    *           見た目 Rviz2 と同じ。
    *           y_map = y_cv::Mat が一致するので処理するには簡単。
    *           map saver の通常の保存時の処理。
    *       false -> map => cv::Mat 変換で、y軸をそのままコピーする。 
    *           見た目 Rviz2 と、上下逆になる。
    *           y_map = map_height - y_cv::Mat - 1 の関係になる。
    * 
    * https://answers.ros.org/question/293890/how-to-use-waitformessage-properly/
    * http://docs.ros.org/en/lunar/api/nav_msgs/html/msg/OccupancyGrid.html
    */
    bool get(bool save_f=false, bool inflation_f=false, bool y_reverse=true, int inf_size=97);

    /*
    * conv_fmt2(nav_msgs::OccupancyGrid_ map_msg)
    */
    void conv_fmt2(std::shared_ptr<const nav_msgs::msg::OccupancyGrid> map);
    void saveMap(const nav_msgs::msg::OccupancyGrid &map,bool save_f=false);
    void saveMapRaw(const nav_msgs::msg::OccupancyGrid &map, bool y_reverse=true, int inf_size=97);
    void check_collision(float x,float y,float &ox,float &oy,int r=5,int func=0);
    int check_obstacle(float x,float y,float rz,float r_lng,int func=0,int black_thresh=15);
    int cource_obstacle_eye(float s_x,float s_y,float d_x,float d_y,float robo_radian_marker,int black_thresh=15,float check_range=0.4);
    void test_plot(float x,float y,float r_yaw,float robot_r=0.3, std::string sub_title="");

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

int check_cource_obstacle_comb(GetMap &get_map,GetMap &get_costmap,float s_x,float s_y,float d_x,float d_y,float robo_radian,int black_thresh);
int check_cource_obstacle_comb_ptr(GetMap *get_map,GetMap *get_costmap,float s_x,float s_y,float d_x,float d_y,float robo_radian,int black_thresh);

#endif      // GET_MAP_HPP
