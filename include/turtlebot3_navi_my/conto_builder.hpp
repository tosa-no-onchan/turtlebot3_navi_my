/*
* ロボットの自由領域を識別して、走行ラインの経路計画を作成します。
*
* https://www.qoosky.io/techs/2798a45608  -> バウンディングボックス、最小内包円 例がある。
* https://docs.opencv.org/3.4/df/d0d/tutorial_find_contours.html
* https://tora-k.com/2019/06/23/binary-opencv/  -> 【C++】OpenCVを用いた画像の二値化プログラム【画像処理】
* http://cvwww.ee.ous.ac.jp/opencv_practice4/  -> OpenCV 画像処理演習 ― 図形描画編
* https://cvtech.cc/centroid/ -> 重心の求め方
*
* conto_builder.hpp
*/

#ifndef __CONTO_BUILDER_HPP__
#define __CONTO_BUILDER_HPP__

#include <string>

#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>
#include <unistd.h>

/*-----------------------
Robo_Slice
------------------------*/
typedef struct{
    int ctl;        // 制御情報
    int blob_n;     // ブロブ番号
    cv::Point f;    // スライスの開始位置
    cv::Point l;    // スライスの終了位置
    cv::Point c;    // スライスの中点
} Robo_Slice;

/*-----------------------
Robo_Slice Compact
------------------------*/
typedef struct{
    //int slice_no;   // スライス番号
    cv::Point f;    // スライスの開始位置
    cv::Point l;    // スライスの終了位置
    //cv::Point c;    // スライスの中点
} Robo_Slice_Compat;

/*-----------------------
Robo_Slice classify table
  ロボットの走行ラインブロブの分類テーブル
------------------------*/
typedef struct{
    int ctl;        // 制御情報 : -1 > 無効
    int blob_n;     // ブロブ番号
    cv::Point g;    // ブロブの重心点
    std::vector<Robo_Slice_Compat> robo_slice_vec;  // このブロブに属する、 Robo_Slice
} Robo_Slice_Clast;


/*-----------------------
Robo_Cource Plan
  ロボットの走行プラン
------------------------*/
typedef struct{
    int sec;        // 順番
    int blob_n;     // ブロブ番号
    float dist;     // ロボットからの距離 -> 単純に、top-left から、 down -> left の順が良いかも
} Cource_Plan;


bool compare_Cource_Plan_dist_min(Cource_Plan &s1,Cource_Plan &s2);
//bool compare_Cource_Plan_dist_min(Cource_Plan &s1,Cource_Plan &s2){
//    return s1.dist < s2.dist;
//}


/*
* Contours Builder
* ロボットの自由領域を識別して、走行ラインの経路計画を作成します。
*  cource_plan_vec_ : ロボット走行プラン が作成される。
*/
class ContoBuilder
{
private:
    cv::Mat map_drawing_;
    std::vector<std::vector<cv::Point>> map_blob_;  // map の　ブロブ

    cv::Mat robo_map_; 
    cv::Mat  map_binary_;

    std::vector<Robo_Slice> robo_slice_;            // ロボットのいる、自由領域の走行スライスライン(開放します。)

public:
    //std::string img_path_="../house_map.pgm";
    //std::string img_path_="../house_map-alt.pgm";
    std::string img_path_="../my_map.pgm";
    //std::string img_path_="/home/nishi/map_builder.pgm";

    double threshold_val_=250;      // RGDカメラの場合、白領域(自由領域)が残るので、250
    //  double threshold_val_=200;      // Stereo Camera で Map すると グレー領域(未知領域) になるので、205 

    cv::Mat  map_gray_;
    std::vector<Robo_Slice_Clast> robo_slice_clast_vec_;     // ロボットの走行ラインの分類テーブル
    std::vector<Cource_Plan> cource_plan_vec_;       // ロボット走行プラン

    int rows_;
    int cols_;

    int robo_radius=4;      // robot raius [dot]   0.20[M] / 0.05[map resolution]
    //int safe_margin=2;      // safty margin [dot] 2*0.05=0.1[M]
    int safe_margin=4;      // safty margin [dot] 4*0.05=0.2[M] changed by nishi 2024.4.7
    //int safe_margin=3;      // safty margin [dot] 3*0.05=0.15[M] changed by nishi 2024.4.7
    bool center_classfi=false;  // 分類基準 true:センター位置を基準にした分類  false:スタート位置を基準にした分類

    int start_x_,start_y_;
    bool isCourceDisp_=true;

    int my_bi_;
    ContoBuilder(){}
    /*
    * void init(std::string pgm_path,double threshold=250)
    *    std::string pgm_path : "xxx.pgm"
    *    double  threshold : 250      // white RGDカメラの場合、白領域(自由領域)が残るので、250
    *                      : 200      // gray Stereo Camera で Map すると グレー領域(未知領域) になるので、205 
    */
    void init(std::string pgm_path,double threshold=250){
        threshold_val_=threshold;
        img_path_= pgm_path;
        map_gray_ = cv::imread(img_path_, cv::IMREAD_GRAYSCALE);
        set_up();
    }

    void set_up(){

        rows_ = map_gray_.rows;     // 行
        cols_ = map_gray_.cols;     // 列

        cv::Mat tmp_map_blur;

        // エッジ検出はノイズの影響を受けやすいため平滑化しておきます。
        cv::blur(map_gray_, tmp_map_blur, cv::Size(3,3));   // rgbd の場合か
        //    cv::blur(map_gray_, tmp_map_blur, cv::Size(2,2));   // stereo Camera の場合か

        // 2値化データを使います。
        //cv::threshold(map_gray_,map_binary_,250,255,cv::THRESH_BINARY);
        // 白領域(自由領域)の場合 254
        // グレー領域(未知領域) -> Stereo Camera で Map すると、205 にする。
        cv::threshold(tmp_map_blur,map_binary_,threshold_val_,255,cv::THRESH_BINARY);

        // cv::findContours は第一引数を破壊的に利用するため imshow 用に別変数を用意しておきます。
        //cv::Mat binary2 = binary_.clone();

        std::vector<cv::Vec4i> tmp_hierarchy;
        // cv::Point の配列として、輪郭を計算します。
        cv::findContours(map_binary_, map_blob_, tmp_hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        tmp_hierarchy.clear();

        std::cout << map_blob_.size() << std::endl; //=> 36

        //std::cout << contours[contours.size() - 1][0] << std::endl; //=> [154, 10]

        //printf("%s",contours[0]);
        //auto ss = contours[0];
        //print(ss);
        //std::vector<cv::Point_<int>> v = contours[0];
        //printf("%ld",v.size());


        // 輪郭を可視化してみます。分かりやすさのため、乱数を利用して色付けします。
        map_drawing_ = cv::Mat::zeros(map_gray_.size(), CV_8UC3);

        cv::RNG rng(12345);
        cv::Scalar color = cv::Scalar(rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256));

        //#define SETUP_TEST1
        #if defined(SETUP_TEST1)
            cv::imshow("map_gray", map_gray_);
            for( size_t i = 0; i< map_blob_.size(); i++ ) {
                cv::Scalar color = cv::Scalar(rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256));
                cv::drawContours(map_drawing_, map_blob_, (int)i, color);
                cv::imshow("drawing", map_drawing_);
                cv::waitKey(300);
            }
            cv::imshow("map_drawing", map_drawing_);
            cv::waitKey(100);
        #endif

        //#define TEST3_VIEW
        #if defined(TEST3_VIEW)
            cv::Mat map_drawing_chk = cv::Mat::zeros(map_gray_.size(), CV_8UC3);
        #endif

        //cv::imshow("gray", gray_);
        //cv::imshow("blur", blur_);
        for( size_t i = 0; i< map_blob_.size(); i++ ) {
            cv::Scalar color = cv::Scalar(rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256));
            // map_blob_[i] の i が、 cv::Scalar(r, g, b) の値からすぐ求まるように割り当てる。
            //  ピクセルの色から、 i を逆算できるようにします。
            u_char r,g,b;
            int ix=i+1;
            r = ix/(256*256);
            g = ix % (256*256)/256;
            b = ix % 256;
            // http://cvwww.ee.ous.ac.jp/opencv_practice4/
            //cv::fillPoly(drawing, contours[i], cv::Scalar(0, 255, 255), cv::LINE_AA);
            long int j=map_blob_[i].size();
            //std::cout << "i:"<< i <<" cnt:"<<  j << std::endl; //=> 36
            cv::fillPoly(map_drawing_, map_blob_[i], cv::Scalar(r, g, b), cv::LINE_AA);
            #if defined(TEST3_VIEW)
                cv::fillPoly(map_drawing_chk, map_blob_[i], color, cv::LINE_AA);
                cv::imshow("map_drawing_chk", map_drawing_chk);
                cv::waitKey(300);
            #endif
        }
        #if defined(TEST3_VIEW)
            cv::imshow("map_drawing_chk", map_drawing_chk);
            cv::waitKey(1000);
        #endif
        tmp_map_blur.release();

    }
    ~ContoBuilder(){
        map_gray_.release();
    }

    /* 
    * slice_classify()
    * ロボット走行ラインの分類化(classify)をする。
    * 1) 走行ラインのスライスの中点から、 robot radius+alfa の円、楕円を描画してブロブ化して、
    *     走行ラインのスライスを分類化をする。 -> start 点 が、良いかも
    * 2) 上記ブロブのバウンディングボックスを求め、その重心点を求める。
    *   ロボット位置から上記重心点が近い順に、選んで、ロボットの走行計画を完成させる。
    * 注) 走行計画は、分類された、ブロブ単位で行う。
    * 
    *  -> robo_slice_clast_vec_: ロボットの走行ラインブロブの分類テーブル
    *     cource_plan_vec_: ロボット走行プラン
    */
    void slice_classify(){

        if(robo_slice_clast_vec_.empty() != true){
            robo_slice_clast_vec_.clear();
        }
        if(cource_plan_vec_.empty() != true){
            cource_plan_vec_.clear();
        }
        robo_slice_clast_vec_.reserve(50);

        // 1. スライスの中点 をブロブ化する。
        cv::Mat tmp_slice_center=cv::Mat::zeros(map_gray_.size(), CV_8UC1);

        // 1.1 スライス中点に半径 robo_radius*2+2 の円を描く
        for( size_t i = 0; i< robo_slice_.size(); i++ ) {
            // # RotatedRect(中心座標, サイズ(x, y), 回転角度degree)
            //cv::RotatedRect rect1(cv::Point2f( 80, 80), cv::Size(60, 60), 0);
    		cv::RotatedRect rect1(cv::Point2f( robo_slice_[i].c.x, robo_slice_[i].c.y), cv::Size(robo_radius*2+2, robo_radius*2+2), 0);
            // 楕円を描く
            // # ellipse(画像, RotatedRect, 色, 線幅, 連結)
            cv::ellipse(tmp_slice_center, rect1, cv::Scalar(255), -1, cv::LINE_AA);
        }

        //#define TEST_SLICE_1
        #if defined(TEST_SLICE_1)
            cv::imshow("tmp_slice_center", tmp_slice_center);
            cv::waitKey(1000);
        #endif

        //std::vector<std::vector<cv::Point_<int>>> tmp_slice_blob_vec;
        std::vector<std::vector<cv::Point_<int>>> tmp_center_blob_vec;
        std::vector<cv::Vec4i> tmp_vv_hierarchy;

        // 1.2 上記、円の重なりを利用して、ブロブ化します。
        cv::findContours(tmp_slice_center, tmp_center_blob_vec, tmp_vv_hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        tmp_vv_hierarchy.clear();

        // 1.3 ブロブ化された領域毎に、ブロブ番号を色として使って、描画します。
        cv::Mat tmp_center_blob_assign = cv::Mat::zeros(tmp_slice_center.size(), CV_8UC3);

        //#define TEST_SLICE_2_VIEW
        #if defined(TEST_SLICE_2_VIEW)
            cv::Mat tmp_center_blob_assign_chk = cv::Mat::zeros(tmp_slice_center.size(), CV_8UC3);
            // 輪郭を可視化してみます。分かりやすさのため、乱数を利用して色付けします。
            cv::RNG rng(12345);
            cv::Scalar color = cv::Scalar(rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256));
        #endif
        for( size_t i = 0; i< tmp_center_blob_vec.size(); i++ ) {
            // tmp_center_blob_vec[i] の i が、 cv::Scalar(r, g, b) の値からすぐ求まるように割り当てる。
            //  ピクセルの色から、 i を逆算できるようにします。
            u_char r,g,b;
            int ix=i+1;
            r = ix/(256*256);
            g = ix % (256*256)/256;
            b = ix % 256;
            // http://cvwww.ee.ous.ac.jp/opencv_practice4/
            //cv::fillPoly(drawing, contours[i], cv::Scalar(0, 255, 255), cv::LINE_AA);
            long int j=tmp_center_blob_vec[i].size();
            //std::cout << "i:"<< i <<" cnt:"<<  j << std::endl; //=> 36
            cv::fillPoly(tmp_center_blob_assign, tmp_center_blob_vec[i], cv::Scalar(r, g, b), cv::LINE_AA);
            // 可視化して、チェック
            #if defined(TEST_SLICE_2_VIEW)
                cv::Scalar color = cv::Scalar(rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256));
                cv::fillPoly(tmp_center_blob_assign_chk, tmp_center_blob_vec[i], color, cv::LINE_AA);
                cv::imshow("tmp_center_blob_assign_chk", tmp_center_blob_assign_chk);
                cv::waitKey(300);
            #endif
        }
        #if defined(TEST_SLICE_2_VIEW)
            cv::imshow("tmp_center_blob_assign_chk", tmp_center_blob_assign_chk);
            cv::waitKey(1000);
            tmp_center_blob_assign_chk.release();

        #endif

        // 2. 上記、 tmp_center_blob_assign の色から、robo_slice_[n].blob_n の ブロブ番号のマッチングを行う。
        for( size_t i = 0; i< robo_slice_.size(); i++ ) {
            int x = robo_slice_[i].c.x;
            int y = robo_slice_[i].c.y;
            // (y,x) の位置のピクセルの色を得る
            cv::Vec<unsigned char, 3>col = tmp_center_blob_assign.at<cv::Vec3b>(y,x);
            // そのピクセルの色をから、走行ラインの中点(または、始点)の場所のブロブ番号を逆算する。
            int blob_n = col[0]*(256*256)+col[1]*256+col[2]-1;
            robo_slice_[i].blob_n = blob_n;
            //std::cout << " i:" << i << " blob_n:" << blob_n << std::endl;
        }

        //#define TEST_SLICE_3_VIEW
        #if defined(TEST_SLICE_3_VIEW)
            cv::Mat boundx=cv::Mat::zeros(map_gray_.size(), CV_8UC1);
        #endif

        // 3. スライス中点(または始点)のブロブの(バウンディングボックスを求めて、) その重心を求めて、 
        //   robo_slice_clast_vec_.g.x , g.y にセットする。
        // 注) スライス開始点もある。
        Robo_Slice_Clast robo_slice_clastx;
        robo_slice_clastx.ctl=0;

        for( size_t i = 0; i< tmp_center_blob_vec.size(); i++ ) {
            cv::Rect boundRect;
            boundRect = cv::boundingRect(tmp_center_blob_vec[i]);
            cv::Moments mu = moments(tmp_center_blob_vec[i]);
            cv::Point2f mc = cv::Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );

            //printf("x: %f  y: %f", mc.x, mc.y);
            //std::cout << " i:" << i <<" x:" << mc.x << " y:" << mc.y << std::endl;
            #if defined(TEST_SLICE_3_VIEW)
                cv::rectangle(boundx, boundRect.tl(), boundRect.br(), cv::Scalar(100), 1);
                //cv::circle(boundx, mc, 4, cv::Scalar(100), -1, 4);
                cv::circle(boundx, mc, 2, cv::Scalar(100), -1);
            #endif
            robo_slice_clastx.blob_n=i;
            robo_slice_clastx.g.x=(int)mc.x;    // 重心 x
            robo_slice_clastx.g.y=(int)mc.y;    // 重心 y
            robo_slice_clast_vec_.push_back(robo_slice_clastx);
        }
        #if defined(TEST_SLICE_3_VIEW)
            cv::imshow("boundx", boundx);
            cv::waitKey(1000);
        #endif
        std::cout << " robo_slice_clast_vec_.size():" << robo_slice_clast_vec_.size() << std::endl;

        // 4. robo_slice_clast_vec_[n] に属する、robo_slice_[n] を Robo_Slice_Compat に変換して、チェインする
        for( size_t i = 0; i< robo_slice_.size(); i++ ) {
            int blob_n = robo_slice_[i].blob_n;
            Robo_Slice_Compat slice_compat;
            slice_compat.f = robo_slice_[i].f;
            slice_compat.l = robo_slice_[i].l;
            if(robo_slice_clast_vec_[blob_n].robo_slice_vec.empty() == true){
                // ここが、無駄使いか?
                //robo_slice_clast_vec_[blob_n].robo_slice_vec.reserve(20);
                robo_slice_clast_vec_[blob_n].robo_slice_vec.reserve(5);
            }
            robo_slice_clast_vec_[blob_n].robo_slice_vec.push_back(slice_compat);
        }
        // 検証してみる。
        // robo_slice のメンバーが無いエントリーは、無効にする。
        int b_cnt=0;
        for( size_t i = 0; i< robo_slice_clast_vec_.size(); i++ ) {
            // size() が、 0 のメンバーがあるみたい。
            if(robo_slice_clast_vec_[i].robo_slice_vec.size()==0){
                robo_slice_clast_vec_[i].ctl= -1;
                std::cout << " reject robo_slice_clast_vec_[" << i << "].robo_slice_vec.size()=" << robo_slice_clast_vec_[i].robo_slice_vec.size() << std::endl;
            }
            else
                b_cnt++;
            //std::cout << " robo_slice_clast_vec_[" << i << "].blob_n="<< robo_slice_clast_vec_[i].blob_n << std::endl;
            //std::cout << " robo_slice_clast_vec_[" << i << "].robo_slice_id_vec.size()=" << robo_slice_clast_vec_[i].robo_slice_id_vec.size() << std::endl;
        }


        // 5. 走行プランを作る。
        //   ロボット位置からの距離の近い順に、プランを作る。 -> 左上から、下 - 右 の順にすべき
        cource_plan_vec_.reserve(b_cnt);
        for( size_t i = 0; i< robo_slice_clast_vec_.size(); i++ ) {
            if(robo_slice_clast_vec_[i].ctl != -1){
                Cource_Plan cplan;
                int x = robo_slice_clast_vec_[i].g.x;
                int y = robo_slice_clast_vec_[i].g.y;
                //#define USE_NEARE_ROBO
                #if defined(USE_NEARE_ROBO)
                    // ロボット位置からの距離の近い順
                    cplan.dist = sqrtf(std::pow((float)(x - start_x_), 2.0) + std::pow((float)(y - start_y_), 2.0));
                #else
                    // 左上から、下 - 右 の順
                    // 注) y 軸は、上下逆にする -> 昇順ソートでうまく処理出来ない
                    cplan.dist =(float)(x*rows_ + (rows_ -y));
                #endif
                cplan.blob_n = robo_slice_clast_vec_[i].blob_n;
                cource_plan_vec_.push_back(cplan);
            }
        }
        // 5.1 近い順にする。
        // dist でソート 小さい順
        std::sort(cource_plan_vec_.begin(),cource_plan_vec_.end(),compare_Cource_Plan_dist_min);


        // 描画して、検証してみる
        //#define USE_COURSE_DISP
        //#if defined(USE_COURSE_DISP)
        if(isCourceDisp_==true){
            cv::Mat plan_mat;
            cv::cvtColor(map_gray_,plan_mat,cv::COLOR_GRAY2BGR);

            cv::RNG tmp_rng(12345);

            for( size_t i = 0; i< cource_plan_vec_.size(); i++ ) {
                int blob_n = cource_plan_vec_[i].blob_n;
                std::cout << " cource_plan_vec_[" << i << "].blob_n=" << cource_plan_vec_[i].blob_n << std::endl;
                std::cout << " cource_plan_vec_[" << i << "].dist=" << cource_plan_vec_[i].dist << std::endl;
                cv::Scalar color = cv::Scalar(tmp_rng.uniform(0, 256), tmp_rng.uniform(0,256), tmp_rng.uniform(0,256));
                // 該当 Robo_Slice_Clast を処理
                // ここは、ロボットの今の位置から近い順にしたほうが良いみたい。
                for(size_t j = 0; j < robo_slice_clast_vec_[blob_n].robo_slice_vec.size(); j++){
                    cv::Point f = robo_slice_clast_vec_[blob_n].robo_slice_vec[j].f;
                    cv::Point l = robo_slice_clast_vec_[blob_n].robo_slice_vec[j].l;
                    // 線を引く
                    // # line(画像, 始点座標, 終点座標, 色, 線幅, 連結)
                    //cv::line(image, p1, p2, cv::Scalar(255, 0, 0), lineWidth, lineType);
                    cv::line(plan_mat,f,l,color ,1);
                    cv::imshow("plan_mat", plan_mat);
                    cv::waitKey(350);
                }
                //cv::imshow("plann_mat", plann_mat);
                //cv::waitKey(400);
            }
        }
        //#endif

        tmp_slice_center.release();
        tmp_center_blob_assign.release();

        tmp_center_blob_vec.clear();
    }

    /*
    * void robo_line_slice()
    * 参照
    *  robo_map_
    * ロボット走行ラインをスライス化して、そのスライス線分の中点か、始点を求める
    * 注) スタート点がよいかも。
    * robo_map_ から、ロボット radius*2 幅 の走行ラインをスライス化して求める。
    *   -> robo_slice_ : ロボットのいる、自由領域の走行スライスライン
    */
    void robo_line_slice(){

        if(robo_slice_.empty() != true){
            robo_slice_.clear();
        }
        robo_slice_.reserve(50);

        // 1. ロボットがいるブロブのバウンディングボックス を求めて、robo_top_y、 robo_bottom_y を算出する。
        cv::Mat  tmp_binary,tmp_gray,tmp_line_mask,tmp_lines;
        // グレーに変換
        cv::cvtColor(robo_map_,tmp_gray,cv::COLOR_BGR2GRAY);

        // ２値化します。
        cv::threshold(tmp_gray,tmp_binary,50,255,cv::THRESH_BINARY);

        //cv::imshow("binary", binary);
        //cv::waitKey(0);

        cv::Rect boundRect;
        boundRect = cv::boundingRect(map_blob_[my_bi_]);

        cv::Point_<int> tl = boundRect.tl();
        std::cout << " top_left.x:" << tl.x << " top_left.y:" << tl.y << std::endl;

        //printf("%s",top_left);  
        // cv::Point_<int>

        cv::Point_<int> br = boundRect.br();
        std::cout << " bottom_right.x:" << br.x << " bottom_right.y:" << br.y << std::endl;

        cv::rectangle(robo_map_, boundRect.tl(), boundRect.br(), cv::Scalar(255, 100,0), 1);

        tmp_line_mask=cv::Mat::zeros(tmp_gray.size(), CV_8UC1);

        // 2. ラインを引く領域のみ対象として、走行ラインを引きます。
        //  top_left.y + robo_radius から bottom_right.y - robo_radius の間に robo_radius*2 毎にラインを引きます。
        // 2.1 ラインのマスクを作成
        for(int i = tl.y+robo_radius;i <= br.y-robo_radius; i += robo_radius*2){
			// 線を引く
			// # line(画像, 始点座標, 終点座標, 色, 線幅, 連結)
			//cv::line(image, p1, p2, cv::Scalar(255, 0, 0), lineWidth, lineType);
            cv::line(tmp_line_mask,cv::Point(tl.x,i),cv::Point(br.x,i),cv::Scalar(255),1);
        }

        //cv::imshow("line_mask", line_mask);
        //cv::waitKey(0);

        // 2.2 binary と　ラインのマスク の AND を取って 走行ラインを求める
        cv::bitwise_and(tmp_binary,tmp_line_mask,tmp_lines);

        //#define LINE_SLICE_TEST1
        #if defined(LINE_SLICE_TEST1)
            cv::imshow("tmp_lines", tmp_lines);
            cv::waitKey(1000);
        #endif

        std::vector<std::vector<cv::Point_<int>>> tmp_vv;
        std::vector<cv::Vec4i> tmp_vv_hierarchy;

        // 2.3 上記、走行ラインのブロブを求める
        cv::findContours(tmp_lines, tmp_vv, tmp_vv_hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        tmp_vv_hierarchy.clear();

        std::cout << " tmp_vv.size()=" << tmp_vv.size() << std::endl;

        Robo_Slice robo_slice;
        robo_slice.ctl=0;
        robo_slice.blob_n=-1;

        // robo_lines_ から、ロボットサイズより短い線は取り除く。
        for( size_t i = 0; i< tmp_vv.size(); i++ ) {
            long int j=tmp_vv[i].size();
            //std::cout <<"robo_lines_["<< i <<"].size()=" << j << std::endl;
            cv::Point front = tmp_vv[i].front();
            cv::Point back = tmp_vv[i].back();
            // ロボット幅より短い ラインです
            if((back.x - front.x) < robo_radius*2+safe_margin*2){
                std::cout <<" vv["<< i <<"].length=" << back.x - front.x << std::endl;
            }
            else{
                // 対象となる スライスライン のみを、robo_slice_ へ残す。
                robo_slice.f=tmp_vv[i].front();
                robo_slice.f.x+=safe_margin;    // マージン分ずらす
                robo_slice.l=tmp_vv[i].back();
                robo_slice.l.x -=safe_margin;    // マージン分ずらす
                robo_slice.c.y=tmp_vv[i].front().y;
                int x_center;
                if(center_classfi==true)
                    // スライス線分の中点 (x 軸の中点) を求める
                    x_center = (robo_slice.l.x - robo_slice.f.x)/2 + robo_slice.f.x;
                else
                    // スライス線分の開始点 を使う。
                    x_center = robo_slice.f.x;

                robo_slice.c.x=x_center;
                robo_slice_.push_back(robo_slice);
            }
        }
        tmp_vv.clear();

        tmp_binary.release();
        tmp_gray.release();
        tmp_line_mask.release();
        tmp_lines.release();
    }

    /*
    * ロボットのいる場所のブロブを得る
    *  int x: col
    *  int y: row
    * http://opencv.jp/cookbook/opencv_mat.html
    * https://qiita.com/hmichu/items/0a399d9e3bbf3a2a4454
    * 
    * ロボットの場所のブロブ以外は、ブラックにする。
    */
    void get_bolb(int x,int y,bool isCourceDisp=true){
        isCourceDisp_=isCourceDisp;

        std::cout << "get_bolb() x:" << x << " y:" << y <<std::endl;
        start_x_=x;
        start_y_=y;

        // 行数 y
        std::cout << " rows:" << rows_ <<std::endl;
        // 列数 x
        std::cout << " cols:" << cols_ << std::endl;
        // 次元数
        //std::cout << "dims:" << map_drawing_.dims << std::endl;

        // チャンネル数
        std::cout << " channels:" << map_drawing_.channels() << std::endl;

        if(y >= rows_ || x >= cols_){
            std::cout << " error exceed x or y" << std::endl;
            return;
        }

        // ロボットの位置(y,x) のピクセルの色を得る
        cv::Vec<unsigned char, 3>col = map_drawing_.at<cv::Vec3b>(y,x);
        //printf("%s",col);
        //std::cout << "col: r:" << col[0] << " g:" << col[1] << " b:" << col[2] <<std::endl;
        printf(" col b:%d g:%d r:%d\n",col[0],col[1],col[2]);

        // そのピクセルの色をから、ロボットのいる場所のブロブ番号を逆算する。
        my_bi_ = col[0]*(256*256)+col[1]*256+col[2]-1;

        std::cout << " my_bi_:" << my_bi_ << std::endl;

        if(!(my_bi_>=0 && my_bi_ < map_blob_.size())){
            std::cout << " my_bi_ is not acceptable" << std::endl;
            return;
        }

        // robo_map_ を初期化(黒) 
        robo_map_ = cv::Mat::zeros(map_gray_.size(), CV_8UC3);

        // ロボット位置のリージョンを赤でぬる。
        // 他のリージョンは、黒塗りする。
        // 1) 重なるリージョンは、一番外側から出てくるみたい。
        // 2) threshold_val_=205 で、gray も対象にすると、一番外側に、大きなリージョンが現れてしまう。
        //     ロボットリージョンは、その内ら側に出てくる。
        // 3) 先に、ロボットのリージョンを描いてしまうと、それより大きいブロブがあるから、赤のリージョンを消してしまうみたい。
        for( size_t i = 0; i< map_blob_.size(); i++ ) {
            if(i != my_bi_)
                cv::fillPoly(robo_map_, map_blob_[i], cv::Scalar(0, 0, 0), cv::LINE_AA);
            else
                // ロボット位置のリージョンを赤でぬる。
                cv::fillPoly(robo_map_, map_blob_[my_bi_], cv::Scalar(0, 0, 180), cv::LINE_AA);
        }

        //#define GET_BLOB_TEST1
        #if defined(GET_BLOB_TEST1)
            cv::imshow("robo_map_1", robo_map_);
            cv::waitKey(100);
        #endif

        // ロボット走行ラインのスライス化と中点の算出
        robo_line_slice();

        // ロボット走行ラインの分類化をする。
        slice_classify();

        //#define GET_BLOB_TEST2
        #if defined(GET_BLOB_TEST2)
            // ロボットの場所のマークを表示する。
            // # ellipse(画像, 中心座標, Size(x径, y径), 楕円の回転角度, 始点角度, 終点角度, 色, 線幅, 連結)
            cv::ellipse(robo_map_, cv::Point( x, y), cv::Size(5, 5), 0, 0, 360, cv::Scalar(255, 200, 0), 1, cv::LINE_AA);

            cv::imshow("robo_map_", robo_map_);
            cv::waitKey(100);
        #endif

        robo_map_.release();
        map_binary_.release();

        if(map_blob_.empty() != true)
            map_blob_.clear();
        if(robo_slice_.empty() != true)
            robo_slice_.clear();
    }
};

#endif


