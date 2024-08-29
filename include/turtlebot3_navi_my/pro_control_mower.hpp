/*
* Programable Controller with Auto Mower
*  turtlebot3_navi_my/include/turtlebot3_navi_my/pro_control_mower.hpp
*
* pro_control_map.hpp
*
*/

#ifndef PRO_CONTROL_MOWER_HPP
#define PRO_CONTROL_MOWER_HPP

#include <string>

#include <opencv2/opencv.hpp>
#include <iostream>

#include "conto_builder.hpp"
#include "pro_control.hpp"

/*----------------------------
- class ContoBuilderMower: public ContoBuilder
-  ContoBuilder を継承して、総経路のプランに従って、ロボットを動かす関数を提供する
----------------------------*/
class ContoBuilderMower: public ContoBuilder{
private:
    bool plot_init_=false;
    cv::Mat plann_mat_;
    cv::RNG rng_;

public:
    MapM mapm_;

    void init(cv::Mat &mat_map,MapM mapm,double threshold=250){
        threshold_val_=threshold;
        plot_init_=false;
        //img_path_="/home/nishi/map_builder.pgm";
        //gray_ = cv::imread(img_path_, cv::IMREAD_GRAYSCALE);
        map_gray_=mat_map;
        mapm_=mapm;

        set_up();
    }


    void cource_plot(cv::Point f,cv::Point l){
        // 描画して、検証してみる
        if(plot_init_==false){
            cv::cvtColor(map_gray_,plann_mat_,cv::COLOR_GRAY2BGR);
            //rng_(rng_val);
            rng_(12345);
            plot_init_=true;
        }

        cv::Scalar color = cv::Scalar(rng_.uniform(0, 256), rng_.uniform(0,256), rng_.uniform(0,256));

        cv::line(plann_mat_,f,l,color ,1);
        cv::imshow("plann_mat_", plann_mat_);
        cv::waitKey(100);
        //cv::destroyWindow("plann_mat_");
    }

    /*
    * map2tr_real(int x,int y,float &f_x, float &f_y)
    *    Mat map 座標 -> real world 座標に変換。
    *     contbuilder.init() を実行した時の情報を使うこと。
    *     ratbamap localization:=true で起動しても、 Static Map がリサイズされるから。
    */
    void map2tr_real(int x,int y,float &f_x, float &f_y){
        f_x = ((float)x + 0.5)  * mapm_.resolution + mapm_.origin[0];
        f_y = ((float)y + 0.5)  * mapm_.resolution + mapm_.origin[1];

    }
};


/*----------------------------
- class Programable Controller for Auto Mower
-  build in auto_map() and auto_map_anchor()
----------------------------*/
class ProControlMower: public ProControl
{
private:

    double threshold_;  // add by nishi 2024.4.24
    bool plann_test_;   // add by nishi 2024.4.24
    bool all_nav2_;     // add by nishi 2024.4.24

public:
    ProControlMower(){}
    void init(std::shared_ptr<rclcpp::Node> node);  // add by nishi 2024.4.24
    void auto_mower(int m_type=1);

};


#endif      // MULTI_GOALS_H


