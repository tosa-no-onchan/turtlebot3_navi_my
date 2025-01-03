/*
* Machine Learning Planner
*  turtlebot3_navi_my/include/ml_planner.hpp
*
*/

//#include  "turtlebot3_navi_my/com_def.hpp"
#include "turtlebot3_navi_my/pro_control_sub.hpp"
#include "turtlebot3_navi_my/robot_driveCore.hpp"

#include "rclcpp/rclcpp.hpp"

//#include <cassert>
#include <fstream>
#include <filesystem>
#include <cmath>
#include <sstream>

#include <opencv2/opencv.hpp>

// add by nishi 2024.12.23
#define USE_OPP_TFLITE
#if defined(USE_OPP_TFLITE)
    #include <opp_tflite/opp_tflite.hpp>
#endif

#pragma once

namespace ml_planner{

class MlPlanner{
public:
    MlPlanner(){}
    void init(std::shared_ptr<rclcpp::Node> node, Robot_DriveCore *drive, GetMap *get_gcostmap, Path_plan *path_plan);
    bool make_plann(float dx,float dy, bool make_img=false, bool opp_on=false);
    bool make_plann_(float sx,float sy,float dx,float dy, bool make_img=false, bool opp_on=false);

    void get_bound_rect(cv::Mat &map_mat,int sx_m,int sy_m,int dx_m,int dy_m,cv::Mat &cropped,float h_h = 30);
    void get_bound_rect_old(cv::Mat &map_mat,int sx_m,int sy_m,int dx_m,int dy_m,cv::Mat &cropped,float h_f = 60);

    float ry_ = 1.5; // a half of robo running cource width[M]

    float min_dist_ = 2.0;   // cource min distance [M]
    //float max_dist_ = 10.0;  // cource max distance [M]
    float max_dist_ = 13.0;  // cource max distance [M]
    int max_no_=-1;

    bool y_reverse_= true;   // false : map => cv::Mat 変換で、y軸をそのままコピーする。 見た目 Rviz2 と、上下逆になる。使うときは、y を変換要!!。
                            // true : map => cv::Mat 変換で、y軸を逆順にコピーする。見た目 Rviz2 と同じ。y-Map = y-cv::Mat の関係になる。
    int inf_size_=97;
    //int inf_size_=98;

    // add by nishi 2024.12.23
    #if defined(USE_OPP_TFLITE)
        opp_tflite::Opp_Tflite opp_tfl_;
        opp_tflite::Settings settings_;
    #endif

    Path_plan *path_plan_;

private:
    std::shared_ptr<rclcpp::Node> node_;
    GetMap *get_gcostmap_;
    Robot_DriveCore *drive_;

    bool is_data_path_ok_=false;

    std::string data_path_="/home/nishi/colcon_ws/src/turtlebot3_navi_my/ml_data/image";

};

}