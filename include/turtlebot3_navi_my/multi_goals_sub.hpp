/*
* multi_goals_sub.hpp

*/

#ifndef MULTI_GOALS_SUB_HPP
#define MULTI_GOALS_SUB_HPP


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
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/transform_datatypes.h"


#include "nav_msgs/msg/map_meta_data.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"



#include "com_def.hpp"
#include "com_lib.hpp"
//#include "Labeling.hpp"


#include <opencv2/opencv.hpp>


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

    std::string _map_frame;

    //nav_msgs::MapMetaData map_info;

    float resolution;
    int free_thresh;
    double org_x,org_y;
    //int x_size,y_size; 

    //Grid grid_;

    bool init_ok=false;

public:

    cv::Mat mat_map_;
    cv::Mat mat_bin_map_;   // map 障害物の　2値化
    cv::Mat mat_bin_free_map_;  // map 非障害物の 2値化
    //Yaml yaml_;
    MapM mapm_;

    GetMap(){}

    //void init(ros::NodeHandle &nh,std::string map_frame="map");
    void init(std::shared_ptr<rclcpp::Node> node,std::string map_frame="map");
 
    /*
    * https://answers.ros.org/question/293890/how-to-use-waitformessage-properly/
    * http://docs.ros.org/en/lunar/api/nav_msgs/html/msg/OccupancyGrid.html
    */
    void get();

    /*
    * conv_fmt2(nav_msgs::OccupancyGrid_ map_msg)
    */
    //void conv_fmt2(boost::shared_ptr<const nav_msgs::OccupancyGrid_<std::allocator<void>>> map);
    void conv_fmt2(std::shared_ptr<const nav_msgs::msg::OccupancyGrid> map);

    //void saveMap(boost::shared_ptr<const nav_msgs::OccupancyGrid_<std::allocator<void>>> map);
    void saveMap(std::shared_ptr<const nav_msgs::msg::OccupancyGrid> map);

    void check_collision(float x,float y,float &ox,float &oy,int func=0);

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

    void init(nav_msgs::MapMetaData map_info,int line_w,std::vector<int8_t> data);
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

#endif      // GET_MAP_HPP
