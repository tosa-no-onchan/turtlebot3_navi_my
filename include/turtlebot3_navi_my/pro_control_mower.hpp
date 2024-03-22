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

#include "pro_control.hpp"



/*
* Contours Builder
*/
/*
* Contours Builder
*/
class ContoBuilder
{
private:
    //std::string img_path_="../house_map.pgm";
    //std::string img_path_="../house_map-alt.pgm";
    std::string img_path_="/home/nishi/map_builder.pgm";
    cv::Mat  binary_,blur_,gray_;
    cv::Mat drawing_;

    std::vector<std::vector<cv::Point> > contours_;
    std::vector<cv::Vec4i> hierarchy_;

public:
    cv::Mat robo_map_; 

    int my_bi_;
    ContoBuilder(){}
    void init(){
        gray_ = cv::imread(img_path_, cv::IMREAD_GRAYSCALE);

        // エッジ検出はノイズの影響を受けやすいため平滑化しておきます。
        cv::blur(gray_, blur_, cv::Size(3,3));
        //cv::blur(gray, blur, cv::Size(2,2));

        // 2値化データを使います。
        //cv::threshold(gray,binary,250,255,cv::THRESH_BINARY);
        cv::threshold(blur_,binary_,250,255,cv::THRESH_BINARY);

        // cv::findContours は第一引数を破壊的に利用するため imshow 用に別変数を用意しておきます。
        //cv::Mat binary2 = binary_.clone();


        // cv::Point の配列として、輪郭を計算します。
        //cv::findContours(canny, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        cv::findContours(binary_, contours_, hierarchy_, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        std::cout << contours_.size() << std::endl; //=> 36

        //std::cout << contours[contours.size() - 1][0] << std::endl; //=> [154, 10]

        //printf("%s",contours[0]);
        //auto ss = contours[0];
        //print(ss);
        //std::vector<cv::Point_<int>> v = contours[0];
        //printf("%ld",v.size());


        // 輪郭を可視化してみます。分かりやすさのため、乱数を利用して色付けします。
        drawing_ = cv::Mat::zeros(gray_.size(), CV_8UC3);
        cv::RNG rng(12345);

        cv::Scalar color = cv::Scalar(rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256));

        //#define TEST1
        #if defined(TEST1)
            cv::imshow("gray", gray_);
            for( size_t i = 0; i< contours_.size(); i++ ) {
                cv::Scalar color = cv::Scalar(rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256));
                cv::drawContours(drawing_, contours_, (int)i, color);
                cv::imshow("drawing", drawing_);
                cv::waitKey(300);
            }
            cv::imshow("drawing", drawing_);
            cv::waitKey(0);
        #endif

        #define TEST3
        #if defined(TEST3)
            //cv::imshow("gray", gray_);
            //cv::imshow("blur", blur_);
            for( size_t i = 0; i< contours_.size(); i++ ) {
                cv::Scalar color = cv::Scalar(rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256));
                u_char r,g,b;
                int ix=i+1;
                r = ix/(256*256);
                g = ix % (256*256)/256;
                b = ix % 256;
                // http://cvwww.ee.ous.ac.jp/opencv_practice4/
                //cv::fillPoly(drawing, contours[i], cv::Scalar(0, 255, 255), cv::LINE_AA);
                long int j=contours_[i].size();
                //std::cout << "i:"<< i <<" cnt:"<<  j << std::endl; //=> 36
                //cv::fillPoly(drawing_, contours_[i], color, cv::LINE_AA);
                cv::fillPoly(drawing_, contours_[i], cv::Scalar(r, g, b), cv::LINE_AA);
                //cv::imshow("drawing", drawing);
                //cv::waitKey(300);
            }
            //cv::imshow("drawing", drawing_);
            //cv::waitKey(0);
        #endif

    }
    ~ContoBuilder(){
        gray_.release();
        binary_.release();
        blur_.release();
    }

    /*
    * ロボットのいる場所のブロブを得る
    *
    *  int x: col
    *  int y: row
    * http://opencv.jp/cookbook/opencv_mat.html
    * https://qiita.com/hmichu/items/0a399d9e3bbf3a2a4454
    */
    void get_bolb(int x,int y){

        std::cout << "get_bolb() x:" << x << " y:" << y <<std::endl;

        robo_map_ = drawing_.clone();

        int rows = robo_map_.rows;
        int cols = robo_map_.cols;

        // 行数 y
        std::cout << "rows:" << rows <<std::endl;
        // 列数 x
        std::cout << "cols:" << cols << std::endl;
        // 次元数
        //std::cout << "dims:" << robo_map_.dims << std::endl;

        // チャンネル数
        std::cout << "channels:" << robo_map_.channels() << std::endl;

        if(y >= rows || x >= cols){
            std::cout << " error exceed x or y" << std::endl;
            return;
        }


        cv::Vec<unsigned char, 3>col = robo_map_.at<cv::Vec3b>(y,x);
        //printf("%s",col);
        //std::cout << "col: r:" << col[0] << " g:" << col[1] << " b:" << col[2] <<std::endl;
        printf("col b:%d g:%d r:%d\n",col[0],col[1],col[2]);

        my_bi_ = col[0]*(256*256)+col[1]*256+col[2]-1;

        std::cout << "my_bi_:" << my_bi_ << std::endl;

        cv::Mat drawingx = cv::Mat::zeros(gray_.size(), CV_8UC3);

        // b,g,r
        cv::fillPoly(drawingx, contours_[my_bi_], cv::Scalar(0, 0, 180), cv::LINE_AA);

		// # ellipse(画像, 中心座標, Size(x径, y径), 楕円の回転角度, 始点角度, 終点角度, 色, 線幅, 連結)
		cv::ellipse(drawingx, cv::Point( x, y), cv::Size(5, 5), 0, 0, 360, cv::Scalar(255, 200, 0), 1, cv::LINE_AA);

        cv::imshow("drawingx", drawingx);
        cv::waitKey(0);

    }
};




/*----------------------------
- class Programable Controller for Auto Mower
-  build in auto_map() and auto_map_anchor()
----------------------------*/
class ProControlMower: public ProControl
{
private:


public:
    ProControlMower(){}
    void auto_mower();

};


#endif      // MULTI_GOALS_H


