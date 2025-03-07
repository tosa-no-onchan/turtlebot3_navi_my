/*
* turtlebot3_navi_my/src/pro_control_sub.cpp
*
*/

#include "turtlebot3_navi_my/pro_control_sub.hpp"

//using namespace std::chrono_literals;

using std::placeholders::_1;

#include <stdlib.h>

int compare_int(const void *a, const void *b)
{
    return *(int*)a - *(int*)b;
}

bool compare_Gpoint_dist_min(Gpoint &s1,Gpoint &s2){
    return s1.dist < s2.dist;
}

bool compare_Gpoint_dist_max(Gpoint &s1,Gpoint &s2){
    return s1.dist > s2.dist;
}


void condense_Gpoint(std::vector<Gpoint> *gp){
    int size=gp->size();
    std::cout << "#1 gp->size()=" << gp->size() << std::endl;
    std::vector<Gpoint> gp_w;
    gp_w.reserve(size);
    //Gpoint gpx;
    for(int i=0;i<size;i++){
        gp_w.push_back(gp->at(i));
    }
    gp->clear();
    gp->reserve(size);
    for(int i=0;i<size;i++){
        gp->push_back(gp_w.at(i));
    }
    //gp->resize(size);
    //gp->shrink_to_fit();
    std::cout << "#2 gp->size()=" << gp->size() << std::endl;
}

/*-------------------------
* find_Gpoint()
--------------------------*/
bool find_Gpoint(float x,float y,std::vector<Gpoint> &gp){
    for (int i=0;i<gp.size();i++){
        if(gp.at(i).x == x && gp.at(i).y == y){
            return true;
        }
    }
    return false;
}

/*-------------------------
* check_cource_obstacle_comb()
*  get_map と get_costmap の障害物を合成して、
*  走行予定コース上の、Mapの障害物を、robo_radian*2 幅で、チェックする。
*  GetMap &get_map: static map GetMap
*  GetMap &get_costmap: local cost map GetMap
*/
int check_cource_obstacle_comb(GetMap &get_map,GetMap &get_costmap,float s_x,float s_y,float d_x,float d_y,float robo_radian,int black_thresh){

    cv::Mat result,result2,mask;

    std::cout << "start check_cource_obstacle_comb()" << std::endl;

    std::cout << " get_map.mapm_.origin[0]:" << get_map.mapm_.origin[0] <<" get_map.mapm_.origin[1]:" << get_map.mapm_.origin[1] << std::endl;
    // get_map.mapm_.origin[0]:-8.05 get_map.mapm_.origin[1]:-5.8
    // map->info.width=321 map->info.height=232 map->info.resolution=0.05

    std::cout << " get_costmap.mapm_.origin[0]:" << get_costmap.mapm_.origin[0] <<" get_costmap.mapm_.origin[1]:" << get_costmap.mapm_.origin[1] << std::endl;
    // get_costmap.mapm_.origin[0]:-4.5 get_costmap.mapm_.origin[1]:-0.45
    // map->info.width=60 map->info.height=60 map->info.resolution=0.05

    // static map の 原点 p0(左、下) の位置から、cost map の p1(左、下) の off set を求める
    // 注) map topic を cv::Mat に変換時には、Y 軸は、上下逆転している。
    // off_x = p1.x - p0.x
    // off_y = p1.y - p0.y
    int offs_x,offs_y;
    offs_x = (int)((get_costmap.mapm_.origin[0] - get_map.mapm_.origin[0]) / get_map.mapm_.resolution);
    offs_y = (int)((get_costmap.mapm_.origin[1] - get_map.mapm_.origin[1]) / get_map.mapm_.resolution);

    std::cout << " offs_x:" << offs_x <<" offs_y:" << offs_y << std::endl;
    // offs_x:72 offs_y:107
    // これを、static map に drow してみれば、わかる。

    // 注) offs_x, off_y が 負 の時は、cost_map をその分だけ小さくする。

    cv::Mat my_map_bin, my_cost_bin;
    my_map_bin = get_map.mat_bin_map_.clone();
    my_cost_bin = get_costmap.mat_bin_map_.clone();

    //#define CHK_COURCE_OBSTACLE_COMB_TEST1_0
    #if defined(CHK_COURCE_OBSTACLE_COMB_TEST1_0)
        cv::rectangle(my_map_bin, cv::Point(offs_x,offs_y), cv::Point(offs_x+get_costmap.mat_bin_map_.cols,offs_y+get_costmap.mat_bin_map_.rows), cv::Scalar(255,0,0), 1);
        cv::imshow("map.my_map_bin", my_map_bin);
        cv::waitKey(100);
    #endif

    // 【Visual Studio】OpenCVで画像の部分処理（ROI
    // https://qiita.com/kelbird/items/cc19a20840577da43dbe
    // を参考に、static map に cost map を部分的に OR する。
    // ROIは、「元画像のSRCの画像を参照しているだけ」ということを忘れないでください。
    // つまり、元画像が変更されればROI画像も変更されます。また、ROI画像が変更されれば元画像も変更されます。

    // static map で、static map から、はみ出さずに使える部分のを roi とする。
    cv::Mat roi_cost;
    int roi_cost_cols = my_cost_bin.cols;
    int roi_cost_rows = my_cost_bin.rows;
    int roi_cost_x0=0;
    int roi_cost_y0=0;

    // cost map が、 static map からの、はみ出しのチェックと補正をする。
    if(offs_x < 0){
        roi_cost_x0 -= offs_x;
        roi_cost_cols += offs_x;
        offs_x=0;
    }
    if(offs_y < 0){
        roi_cost_y0 -= offs_y;
        roi_cost_rows += offs_y;
        offs_y=0;
    }
    if((offs_x + roi_cost_cols) >= my_map_bin.cols){
        std::cout << " over cost map width:"<< offs_x + roi_cost_cols << " than map width"<< std::endl;
        roi_cost_cols -=(my_map_bin.cols - (offs_x + roi_cost_cols));
    }
    if((offs_y + roi_cost_rows) >= my_map_bin.rows){
        std::cout << " over cost map height:"<< offs_y + roi_cost_rows <<" than map height"<< std::endl;
        roi_cost_rows -=(my_map_bin.rows - (offs_y + roi_cost_rows));
    }

    roi_cost = cv::Mat(my_cost_bin, cv::Rect(roi_cost_x0, roi_cost_y0, roi_cost_cols, roi_cost_rows));   //上部と似たような形での宣言

    // static map 上の cost map が、重なる部分を、roi とする。
    // 画像の部分処理・切り出し
    cv::Mat roi_src;                                    //先に変数を宣言
    roi_src = cv::Mat(my_map_bin, cv::Rect(offs_x, offs_y, roi_cost_cols, roi_cost_rows));   //上部と似たような形での宣言

    //#define CHK_COURCE_OBSTACLE_COMB_ERROR_TEST1
    #if defined(CHK_COURCE_OBSTACLE_COMB_ERROR_TEST1)
        // 動作テスト
        // roi_cost を all 1 にして動作確認します。
        cv::Mat test_all1;
        test_all1 = cv::Mat::ones(roi_cost.cols, roi_cost.rows,CV_8UC1);

        cv::Mat test_all1_bin;
        cv::threshold(test_all1, test_all1_bin, 10, 255, cv::THRESH_BINARY_INV);
        test_all1_bin.copyTo(roi_cost);
    #endif

    // ピクセル毎の論理演算 AND NOT OR XOR
    // https://cvtech.cc/bitwise/

    cv::Mat img_or;
    cv::bitwise_or(roi_src, roi_cost, img_or);

    // cv::Matにおけるclone()とcopyTo()の挙動の違い
    // https://13mzawa2.hateblo.jp/entry/2016/12/09/151205
    img_or.copyTo(roi_src);   // img_or を ROIにコピー


    // real world 座標を、Mat map 座標に変換 
    int s_px = (int)((s_x - get_map.mapm_.origin[0]) / get_map.mapm_.resolution);
    int s_py = (int)((s_y - get_map.mapm_.origin[1]) / get_map.mapm_.resolution);

    int d_px = (int)((d_x - get_map.mapm_.origin[0]) / get_map.mapm_.resolution);
    int d_py = (int)((d_y - get_map.mapm_.origin[1]) / get_map.mapm_.resolution);

    int p_robo_radian = (int)(robo_radian/get_map.mapm_.resolution);

    // Mask画像 を作成
    mask = cv::Mat::zeros(get_map.mapm_.height, get_map.mapm_.width, CV_8UC1);

    // 直線を引く。
    cv::Point sp(s_px, s_py);	// 始点座標(x,y)
    cv::Point dp(d_px, d_py);	// 終点座標(x,y)

    // 両端を少し、扁平にしたいけど?
    cv::line(mask, sp, dp, cv::Scalar(255), p_robo_radian*2, cv::LINE_AA);

    //#define CHK_COURCE_OBSTACLE_COMB_TEST1
    #if defined(CHK_COURCE_OBSTACLE_COMB_TEST1)
        cv::imshow("map.my_map_bin", my_map_bin);
        cv::waitKey(100);
        //cv::destroyAllWindows();
        cv::imshow("cost.mat_bin_map_", get_costmap.mat_bin_map_);
        cv::waitKey(100);
    #endif

    #define CHK_COURCE_OBSTACLE_COMB_TEST2
    #if defined(CHK_COURCE_OBSTACLE_COMB_TEST2)
        cv::imshow("mask", mask);
        cv::waitKey(100);
        //cv::destroyAllWindows();
    #endif

    // 障害物 2値化画像に 円のマスクを実施
    //get_map.mat_bin_map_.copyTo(result2,mask);
    my_map_bin.copyTo(result2,mask);

    #if defined(CHK_COURCE_OBSTACLE_COMB_TEST2)
        cv::imshow("result2", result2);
        cv::waitKey(100);
        //cv::destroyAllWindows();
    #endif

    // 黒色領域の面積(ピクセル数)を計算する
    int black_cnt = cv::countNonZero(result2);

    std::cout << " black_cnt=" << black_cnt;

    if(black_cnt <= black_thresh){
        black_cnt=0;
    }
    std::cout << " adjust black_cnt=" << black_cnt << std::endl;

    //#define TEST_WAIT_xx
    #if defined(TEST_WAIT_xx)
        while(1){
            cv::imshow("result2", result2);
            int key = cv::waitKey(100);
            // esc key
            if (key == 27){
                exit(0);
            }
        }
    #endif

    return black_cnt;
}
/*-------------------------
* check_cource_obstacle_comb()
*  get_map と get_costmap の障害物を合成して、
*  走行予定コース上の、Mapの障害物を、robo_radian*2 幅で、チェックする。
*  GetMap &get_map: static map GetMap
*  GetMap &get_costmap: local cost map GetMap
*/
int check_cource_obstacle_comb_ptr(GetMap *get_map,GetMap *get_costmap,float s_x,float s_y,float d_x,float d_y,float robo_radian,int black_thresh){

    cv::Mat result,result2,mask;

    std::cout << "start check_cource_obstacle_comb_ptr()" << std::endl;

    std::cout << " get_map.mapm_.origin[0]:" << get_map->mapm_.origin[0] <<" get_map.mapm_.origin[1]:" << get_map->mapm_.origin[1] << std::endl;
    // get_map.mapm_.origin[0]:-8.05 get_map.mapm_.origin[1]:-5.8
    // map->info.width=321 map->info.height=232 map->info.resolution=0.05

    std::cout << " get_costmap.mapm_.origin[0]:" << get_costmap->mapm_.origin[0] <<" get_costmap.mapm_.origin[1]:" << get_costmap->mapm_.origin[1] << std::endl;
    // get_costmap.mapm_.origin[0]:-4.5 get_costmap.mapm_.origin[1]:-0.45
    // map->info.width=60 map->info.height=60 map->info.resolution=0.05

    // static map の 原点 p0(左、下) の位置から、cost map の p1(左、下) の off set を求める
    // 注) map topic を cv::Mat に変換時には、Y 軸は、上下逆転している。
    // off_x = p1.x - p0.x
    // off_y = p1.y - p0.y
    int offs_x,offs_y;
    offs_x = (int)((get_costmap->mapm_.origin[0] - get_map->mapm_.origin[0]) / get_map->mapm_.resolution);
    offs_y = (int)((get_costmap->mapm_.origin[1] - get_map->mapm_.origin[1]) / get_map->mapm_.resolution);

    std::cout << " offs_x:" << offs_x <<" offs_y:" << offs_y << std::endl;
    // offs_x:72 offs_y:107
    // これを、static map に drow してみれば、わかる。

    // 注) offs_x, off_y が 負 の時は、cost_map をその分だけ小さくする。

    cv::Mat my_map_bin, my_cost_bin;
    my_map_bin = get_map->mat_bin_map_.clone();
    my_cost_bin = get_costmap->mat_bin_map_.clone();

    //#define CHK_COURCE_OBSTACLE_COMB_TEST1_0
    #if defined(CHK_COURCE_OBSTACLE_COMB_TEST1_0)
        cv::rectangle(my_map_bin, cv::Point(offs_x,offs_y), cv::Point(offs_x+get_costmap->mat_bin_map_.cols,offs_y+get_costmap->mat_bin_map_.rows), cv::Scalar(255,0,0), 1);
        cv::imshow("map.my_map_bin", my_map_bin);
        cv::waitKey(100);
    #endif

    // 【Visual Studio】OpenCVで画像の部分処理（ROI
    // https://qiita.com/kelbird/items/cc19a20840577da43dbe
    // を参考に、static map に cost map を部分的に OR する。
    // ROIは、「元画像のSRCの画像を参照しているだけ」ということを忘れないでください。
    // つまり、元画像が変更されればROI画像も変更されます。また、ROI画像が変更されれば元画像も変更されます。

    // static map で、static map から、はみ出さずに使える部分のを roi とする。
    cv::Mat roi_cost;
    int roi_cost_cols = my_cost_bin.cols;
    int roi_cost_rows = my_cost_bin.rows;
    int roi_cost_x0=0;
    int roi_cost_y0=0;

    // cost map が、 static map からの、はみ出しのチェックと補正をする。
    if(offs_x < 0){
        roi_cost_x0 -= offs_x;
        roi_cost_cols += offs_x;
        offs_x=0;
    }
    if(offs_y < 0){
        roi_cost_y0 -= offs_y;
        roi_cost_rows += offs_y;
        offs_y=0;
    }
    if((offs_x + roi_cost_cols) >= my_map_bin.cols){
        std::cout << " over cost map width:"<< offs_x + roi_cost_cols << " than map width"<< std::endl;
        roi_cost_cols -=(my_map_bin.cols - (offs_x + roi_cost_cols));
    }
    if((offs_y + roi_cost_rows) >= my_map_bin.rows){
        std::cout << " over cost map height:"<< offs_y + roi_cost_rows <<" than map height"<< std::endl;
        roi_cost_rows -=(my_map_bin.rows - (offs_y + roi_cost_rows));
    }

    roi_cost = cv::Mat(my_cost_bin, cv::Rect(roi_cost_x0, roi_cost_y0, roi_cost_cols, roi_cost_rows));   //上部と似たような形での宣言

    // static map 上の cost map が、重なる部分を、roi とする。
    // 画像の部分処理・切り出し
    cv::Mat roi_src;                                    //先に変数を宣言
    roi_src = cv::Mat(my_map_bin, cv::Rect(offs_x, offs_y, roi_cost_cols, roi_cost_rows));   //上部と似たような形での宣言

    //#define CHK_COURCE_OBSTACLE_COMB_ERROR_TEST1
    #if defined(CHK_COURCE_OBSTACLE_COMB_ERROR_TEST1)
        // 動作テスト
        // roi_cost を all 1 にして動作確認します。
        cv::Mat test_all1;
        test_all1 = cv::Mat::ones(roi_cost.cols, roi_cost.rows,CV_8UC1);

        cv::Mat test_all1_bin;
        cv::threshold(test_all1, test_all1_bin, 10, 255, cv::THRESH_BINARY_INV);
        test_all1_bin.copyTo(roi_cost);
    #endif

    // ピクセル毎の論理演算 AND NOT OR XOR
    // https://cvtech.cc/bitwise/

    cv::Mat img_or;
    cv::bitwise_or(roi_src, roi_cost, img_or);

    // cv::Matにおけるclone()とcopyTo()の挙動の違い
    // https://13mzawa2.hateblo.jp/entry/2016/12/09/151205
    img_or.copyTo(roi_src);   // img_or を ROIにコピー

    // real world 座標を、Mat map 座標に変換 
    int s_px = (int)((s_x - get_map->mapm_.origin[0]) / get_map->mapm_.resolution);
    int s_py = (int)((s_y - get_map->mapm_.origin[1]) / get_map->mapm_.resolution);

    int d_px = (int)((d_x - get_map->mapm_.origin[0]) / get_map->mapm_.resolution);
    int d_py = (int)((d_y - get_map->mapm_.origin[1]) / get_map->mapm_.resolution);

    int p_robo_radian = (int)(robo_radian/get_map->mapm_.resolution);

    // Mask画像 を作成
    mask = cv::Mat::zeros(get_map->mapm_.height, get_map->mapm_.width, CV_8UC1);

    // 直線を引く。
    cv::Point sp(s_px, s_py);	// 始点座標(x,y)
    cv::Point dp(d_px, d_py);	// 終点座標(x,y)

    // 両端を少し、扁平にしたいけど?
    cv::line(mask, sp, dp, cv::Scalar(255), p_robo_radian*2, cv::LINE_AA);

    //#define CHK_COURCE_OBSTACLE_COMB_TEST1
    #if defined(CHK_COURCE_OBSTACLE_COMB_TEST1)
        cv::imshow("map.my_map_bin", my_map_bin);
        cv::waitKey(100);
        //cv::destroyAllWindows();
        cv::imshow("cost.mat_bin_map_", get_costmap->mat_bin_map_);
        cv::waitKey(100);
    #endif

    #define CHK_COURCE_OBSTACLE_COMB_TEST2
    #if defined(CHK_COURCE_OBSTACLE_COMB_TEST2)
        cv::imshow("mask", mask);
        cv::waitKey(100);
        //cv::destroyAllWindows();
    #endif

    // 障害物 2値化画像に 円のマスクを実施
    //get_map.mat_bin_map_.copyTo(result2,mask);
    my_map_bin.copyTo(result2,mask);

    #if defined(CHK_COURCE_OBSTACLE_COMB_TEST2)
        cv::imshow("result2", result2);
        cv::waitKey(100);
        //cv::destroyAllWindows();
    #endif

    // 黒色領域の面積(ピクセル数)を計算する
    int black_cnt = cv::countNonZero(result2);

    std::cout << " black_cnt=" << black_cnt;

    if(black_cnt <= black_thresh){
        black_cnt=0;
    }
    std::cout << " adjust black_cnt=" << black_cnt << std::endl;

    //#define TEST_WAIT_xx
    #if defined(TEST_WAIT_xx)
        while(1){
            cv::imshow("result2", result2);
            int key = cv::waitKey(100);
            // esc key
            if (key == 27){
                exit(0);
            }
        }
    #endif

    return black_cnt;
}


#if defined(USE_CHECK_COLL)
/*-------------------------
* check_collision()
* float x: 基本座標 x
* float y: 基本座標 y
* float &ox: 補正された基本座標 x
* float &oy: 補正された基本座標 x
* 障害物から適切な距離を求める
*  func=0
*    障害物の2値画像を用いて、障害物の重心から離れる
*    cv::Mat& mat_bin_map : 障害物を 2値化した cv:::Mat
*  func=1
*    非障害物 2値画像を用いて、非障害物の重心の方へ近づく
*    cv::Mat& mat_bin_map : 非障害物を 2値化した cv:::Mat
--------------------------*/
void check_collision(float x,float y,float &ox,float &oy,cv::Mat& mat_bin_map,MapM& mapm,int func){
    ox=x;
    oy=y;

    cv::Point center_p; // 円の中心位置
    int r =5;      // 円の半径  30 -> 1.5[M]      7 ->  0.35[M]  5->0.25[M]
    if(func!=0){
        r = 7;
    }
    cv::Mat result,result2,mask;

    //x0 = (x_g + 0.5) * (double)line_w_ * yaml.resolution + yaml.origin[0];
    //y0 = (y_g + 0.5) * (double)line_w_ * yaml.resolution + yaml.origin[1];

    // 基本座標を、Mat map 座標に変換 
    int px = (int)((x - mapm.origin[0]) / mapm.resolution);
    int py = (int)((y - mapm.origin[1]) / mapm.resolution);

    //ロボットの移動先を、マスクの中心にします。
    center_p.x = px;
    center_p.y = py;

    // Mask画像 を作成
    //mask = cv::Mat::zeros(mat_bin_map_.rows, mat_bin_map_.cols, CV_8UC1);
    mask = cv::Mat::zeros(mapm.height, mapm.width, CV_8UC1);

    // ロボットの移動先を中心にした、半径r の円を描きます。
    cv::circle(mask, center_p, r, cv::Scalar(255),-1);

    // 障害物から離れる処理
    if(func==0){
        // 障害物 2値化画像に 円のマスクを実施
        mat_bin_map.copyTo(result2,mask);

        // 白色領域の面積(ピクセル数)を計算する
        int white_cnt = cv::countNonZero(result2);

        // 障害物と接しています。
        if(white_cnt > 0){

            std::cout << "white_cnt=" << white_cnt << std::endl;

            // 4. 見つかった障害物の重心を求めます。
            cv::Moments m = cv::moments(result2,true);
            // 重心
            double x_g = m.m10 / m.m00;
            double y_g = m.m01 / m.m00;

            std::cout << "x_g=" << x_g <<" y_g="<< y_g << std::endl;

            // map 重心を、基本座標にします。
            float x_gb = ((float)x_g + 0.5) * mapm.resolution + mapm.origin[0];
            float y_gb = ((float)y_g + 0.5) * mapm.resolution + mapm.origin[1];
            x_gb = round_my<float>(x_gb,3);
            y_gb = round_my<float>(y_gb,3);

            x = round_my<float>(x,3);
            y = round_my<float>(y,3);

            // 5. 重心とロボットの移動先 を通る直線を求めて、その直線上を、重心から遠ざかる方向に、
            // ロボットの移動先を 5[cm]移動させます。

            // 5.1 (x_gb,y_gb) から、 (x,y) の角度[rad] を求める
            // 自分からの目的地の方角
            float off_target_x = x - x_gb;
            float off_target_y = y - y_gb;

            //同一地点
            if(fabsf(off_target_x) == 0.0 && fabsf(off_target_y) ==0.0){
                // ここは、将来、非障害物の2値化と Mask を取って、今度は、その重心方向に
                // ロボットの移動先を N[cm]移動させます。
                return;
            }

            float theta_r = std::atan2(off_target_y,off_target_x);   //  [ragian]

            float dx,dy;
            if(white_cnt < 10){
                // 5.2  (x,y) から、(x_gb,y_gb) とは逆方向に 10[cm] ずらす。
                dx = std::cos(theta_r) * 0.1;
                dy = std::sin(theta_r) * 0.1;
            }
            else{
                // 5.2  (x,y) から、(x_gb,y_gb) とは逆方向に 20[cm] ずらす。
                dx = std::cos(theta_r) * 0.2;
                dy = std::sin(theta_r) * 0.2;
            }

            ox=x+dx;
            oy=y+dy;

            std::cout << "ox=" << ox <<" oy="<< oy << std::endl;
        }
    }
    // 非障害物へ近づく処理
    else{
        // 非障害物 2値化画像に 円のマスクを実施
        mat_bin_map.copyTo(result2,mask);

        // 白色領域の面積(ピクセル数)を計算する
        int white_cnt = cv::countNonZero(result2);

        // 非障害物と接しています。
        if(white_cnt > 0){

            std::cout << "white_cnt=" << white_cnt << std::endl;

            // 4. 見つかった非障害物の重心を求めます。
            cv::Moments m = cv::moments(result2,true);
            // 重心
            double x_g = m.m10 / m.m00;
            double y_g = m.m01 / m.m00;

            std::cout << "x_g=" << x_g <<" y_g="<< y_g << std::endl;

            // map 重心を、基本座標にします。
            float x_gb = ((float)x_g + 0.5) * mapm.resolution + mapm.origin[0];
            float y_gb = ((float)y_g + 0.5) * mapm.resolution + mapm.origin[1];
            x_gb = round_my<float>(x_gb,3);
            y_gb = round_my<float>(y_gb,3);

            x = round_my<float>(x,3);
            y = round_my<float>(y,3);

            // 5. 重心とロボットの移動先 を通る直線を求めて、その直線上を、非障害物の重心に近づく方向に、
            // ロボットの移動先を 30-15[cm]移動させます。

            // 5.1 (x,y) から (x_gb,y_gb) の角度[rad] を求める
            // 自分からの目的地の方角
            float off_target_x = x_gb - x;
            float off_target_y = y_gb - y;

            //同一地点
            if(fabsf(off_target_x) == 0.0 && fabsf(off_target_y) ==0.0){
                // ここは、将来、障害物の2値化画像とのMask を取って、今度は、その重心方向から逆に
                // ロボットの移動先を N[cm]移動させます。
                return;
            }

            float theta_r = std::atan2(off_target_y,off_target_x);   //  [ragian]

            float dx,dy;
            if(white_cnt < 10){
                // 5.2  (x,y) から、(x_gb,y_gb) の方へ 35[cm] ずらす。
                dx = std::cos(theta_r) * 0.35;
                dy = std::sin(theta_r) * 0.35;
            }
            else{
                // 5.2  (x,y) から、(x_gb,y_gb) の方へ 25[cm] ずらす。
                dx = std::cos(theta_r) * 0.25;
                dy = std::sin(theta_r) * 0.25;
            }

            ox=x+dx;
            oy=y+dy;

            std::cout << "ox=" << ox <<" oy="<< oy << std::endl;
        }
    }
}
#endif

void gridToWorld(int gx, int gy, float& wx, float& wy,MapM& mapm,bool y_reverse){
    //wx = ((float)gx + 0.5) * yaml.resolution + yaml.origin[0];
    //wy = ((float)(yaml.img_height-gy) + 0.5) * yaml.resolution + yaml.origin[1];  // y軸が反転していない場合。
    wx = ((float)gx + 0.5) * mapm.resolution + mapm.origin[0];
    if(y_reverse==true){
        wy = ((float)gy + 0.5) * mapm.resolution + mapm.origin[1];      // y軸が反転している場合。 
    }
    else{
        wy = ((float)(mapm.height - gy) + 0.5) * mapm.resolution + mapm.origin[1];      // y軸が反転していない場合。 
    }
}

bool worldToGrid(float wx, float wy,int& gx,int& gy,MapM& mapm,bool y_reverse){
    if (wx < mapm.origin[0] || wy < mapm.origin[1])
        return false;
    gx = (int)((wx - mapm.origin[0]) / mapm.resolution);
    gy = (int)((wy - mapm.origin[1]) / mapm.resolution);
    if (gx < mapm.width && gy < mapm.height)
        return true;
    return false;
}

/*
* bool tf_world2MatMap(float w_x, float w_y,int& m_x,int& m_y,MapM& mapm,bool y_reverse)
*   real world -> Mat map 座標 に変換
*
*   bool y_reverse :  Map -> cv::Mat 変換時の y軸の反転
*           true -> y軸を反転する。 上下反転する。 Rviz2 と同じ表示。通常の変換 
*           false -> y軸そのまま。
*/
bool tf_world2MatMap(float w_x, float w_y,int& m_x,int& m_y,MapM& mapm,bool y_reverse){
    if (w_x < mapm.origin[0] || w_y < mapm.origin[1])
        return false;
    m_x = (int)((w_x - mapm.origin[0]) / mapm.resolution);
    m_y = (int)((w_y - mapm.origin[1]) / mapm.resolution);
    if (m_x < mapm.width && m_y < mapm.height){
        // Map -> cv::Mat の時、y軸を逆転している。
        if(y_reverse==true)
            // cv::Mat の y 軸を逆転させる。
            m_y = mapm.height - m_y - 1;
        return true;
    }
    return false;
}

//---------------------------------
// costmap_2d.cpp のサンプル
//void Costmap2D::mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const
//{
//  wx = origin_x_ + (mx + 0.5) * resolution_;
//  wy = origin_y_ + (my + 0.5) * resolution_;
//}
//
//bool Costmap2D::worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) const
//{
//  if (wx < origin_x_ || wy < origin_y_)
//    return false;
//
//  mx = (int)((wx - origin_x_) / resolution_);
//  my = (int)((wy - origin_y_) / resolution_);
//
//  if (mx < size_x_ && my < size_y_)
//    return true;
//
//  return false;
//}
//------------------------------------


/*-------------------------
* class GetMap
--------------------------*/
//void GetMap::init(ros::NodeHandle &nh,std::string map_frame)
void GetMap::init(std::shared_ptr<rclcpp::Node> node,int func,std::string map_frame,bool is_static_map)
{
    //nh_ = nh;
    node_=node;
    map_frame_ = map_frame;
    func_=func;
    is_static_map_=is_static_map;   // add by nishi 2024.9.26

    std::cout << "GetMap::init() func="<< func << std::endl;
   
    //_sub = nh.subscribe(_map_frame, 1);
    //self.map_info = None
    //self.map_data = None
    //self.grid = None
    //self.resolution = None
    
    _line_w = 5;  // ラインの幅 -> grid size [dot]  0.05[m] * 5 = 25[cm]
    _car_r = 4;    // ロボットの回転径 [dot]
    _match_rviz = true;      // True / Flase  -> Rviz の画像と同じにする / しない

    // 'map' のパブリッシュ間隔は、rtabmap_ros/rtabmap の、map のパブリッシュ間隔に拠る。
    // 定期的に出る場合もあれば、不定期に出る場合もある。

    // test 
    //func_=1;

    // map は、不定期に、publish されている場合は、 call back で、取得する。
    if(func_==0){
        #define GET_MAP_TEST2
        #if defined(GET_MAP_TEST2)
            // reffer from amcl
            //map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            //map_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
            //std::bind(&AmclNode::mapReceived, this, std::placeholders::_1));

            subscript_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
                map_frame, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
                std::bind(&GetMap::topic_callback, this, std::placeholders::_1));

        #else
            subscript_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
                map_frame, 10, std::bind(&GetMap::topic_callback, this, _1));
        #endif
    }

    init_ok=true;   // add by nishi 2024.9.4

}

void GetMap::topic_callback(const nav_msgs::msg::OccupancyGrid & map_msg)
{
    //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    //#define USE_MAP_INT_TRACE
    #if defined(USE_MAP_INT_TRACE)
        std::cout << "GetMap::map_msg.header.frame_id=" << map_msg.header.frame_id << std::endl;
    #endif

    //std::cout << "map_msg.header" << map_msg.header << std::endl; 
    //std::cout << "map_msg.info" << map_msg.info << std::endl;

    //auto info = map_msg.info;
    //auto data = map_msg.data;
    //printf("%s",info);
    //printf("%s",data);
    mtx_.lock();    // add by nishi 2024.9.28
    if(map_ptr_cnt_==0){
        map_ptr_=std::make_shared<nav_msgs::msg::OccupancyGrid>(map_msg);
        map_ptr_cnt_=1;
    }
    else{
        map_ptr_.reset();
        map_ptr_=std::make_shared<nav_msgs::msg::OccupancyGrid>(map_msg);
    }
    mtx_.unlock();  // add by nishi 2024.9.28
}


/*
* get(bool save_f=false,bool costmap_f=false, bool revers_f2=true)
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
* http://docs.ros.org/en/jade/api/map_server/html/map__saver_8cpp_source.html
* https://boostjp.github.io/tips/smart_ptr.html
* https://yomi322.hateblo.jp/entry/2012/04/17/223100
* https://qiita.com/usagi/items/3563ddb01e4eb342485e
*/
bool GetMap::get(bool save_f,bool inflation_f,bool y_reverse, int inf_size){
    int cnt=3;
    // for ROS
    //std::shared_ptr<const nav_msgs::msg::OccupancyGrid> map=nullptr;
    // for ROS2
    nav_msgs::msg::OccupancyGrid map;
    bool is_successful = false;

    auto timeout = std::chrono::milliseconds(1000);

    // map が定期的に、publish されている場合。
    if(func_ != 0){
        while (is_successful==false && cnt >0){
            // auto map_msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map",nh_,ros::Duration(1.0));
            //printf("%s",map_msg);  // コンパイルエラーで、型が判る
            //boost::shared_ptr<const nav_msgs::OccupancyGrid_<std::allocator<void>>>

            // ros2 版は、? 無い。自分で作れとの事。 by nishi 2023.2.7
            // https://answers.ros.org/question/378693/waitformessage-ros2-equivalent/
            //  getLatestMsg() が、近い?
            // https://github.com/ros2/rclcpp/issues/1953
            // 
            //   rclcpp/rclcpp/include/rclcpp/wait_for_message.hpp  <-- これが、今あるみたい。
            //    https://github.com/ros2/rclcpp/blob/8e6a6fb32d8d6a818b483660e326f2c5313b64ae/rclcpp/include/rclcpp/wait_for_message.hpp#L78-L94
            //    https://qiita.com/buran5884/items/9ee9b1608716233a9873
            //
            // for ROS
            //map = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(_map_frame,nh_,ros::Duration(1.0));
            // for ROS2
            is_successful = rclcpp::wait_for_message<nav_msgs::msg::OccupancyGrid>(map, node_, map_frame_, timeout);

            if(is_successful==true)
                break;

            //rate.sleep();
            rclcpp::spin_some(node_);

            cnt--;
        }
    }
    // map は、不定期に、publish されている。
    else if(map_ptr_cnt_!=0){
        mtx_.lock();    // add by nishi 2024.9.28
        nav_msgs::msg::OccupancyGrid *map_dt=map_ptr_.get();
        map = *map_dt;
        mtx_.unlock();    // add by nishi 2024.9.28
        is_successful = true;
    }
    
    if (is_successful == true){
        free_thresh = int(0.196 * 255);

        std::cout << "map->info.width=" << map.info.width;         // 225
        std::cout << " map->info.height=" << map.info.height;       // 141

        resolution = map.info.resolution;
        std::cout << " map->info.resolution=" << resolution << std::endl;        // 0.05

        // ロボット位置
        org_x = map.info.origin.position.x;
        org_y = map.info.origin.position.y;
        std::cout << " org_x=" << org_x;
        std::cout << " org_y=" << org_y;

        std::cout << " free_thresh=" << free_thresh << std::endl;

        //x_size = map->info.width / _line_w;
        //y_size = map->info.height / _line_w;
        
        //grid_.init(map->info,_line_w,map->data);

        //conv_fmt2(map);
        saveMap(map,save_f);

        // get costmap reduced inflation size
        if(inflation_f==true){
            saveMapRaw(map,y_reverse,inf_size);
        }

        //2. 障害物を、2値画像 and 反転します。 add by nishi 2022.8.13
        //int thresh = 120;
        //int thresh = 2;
        int thresh = 10;        // 障害物の値 0-100 / 未知領域:128  / 自由領域:255
        // 障害物 < 10 だけを、白にします。
        cv::threshold(mat_map_, mat_bin_map_, thresh, 255, cv::THRESH_BINARY_INV);
        // 非障害物 >= 10 だけを、白にします。
    	cv::threshold(mat_map_, mat_bin_free_map_, thresh, 255, cv::THRESH_BINARY);

        std::cout << "GetMap ok" << std::endl;

        // BlobFinder call
        //std::cout << "call BlobFinder" << std::endl;
        //blobFinder_.check(mat_map_);
 
    }
    else{
        std::cout << "GetMap::get(): 99 error" << std::endl;
    }
    return is_successful;
}

/*
*　GetMap::saveMap(const nav_msgs::msg::OccupancyGrid &map,bool save_f)
*    reverse_f : true -> map => cv::Mat 変換で、y軸を逆順にコピーする。 
*           見た目 Rviz2 と同じ。
*           y_map = y_cv::Mat が一致するので処理するには簡単。
*           map saver の通常の保存時の処理。
*
* http://docs.ros.org/en/jade/api/map_server/html/map__saver_8cpp_source.html
*
* http://docs.ros.org/en/lunar/api/nav_msgs/html/msg/OccupancyGrid.html
*
*/
//void GetMap::saveMap(boost::shared_ptr<const nav_msgs::OccupancyGrid_<std::allocator<void>>> map){
void GetMap::saveMap(const nav_msgs::msg::OccupancyGrid &map,bool save_f){
    FILE* out;
    FILE* yaml_fp;
    std::string mapname_ ="/home/nishi/map_builder";
    std::string mapdatafile = mapname_ + ".pgm";

    mat_map_ = cv::Mat::zeros(map.info.height,map.info.width,CV_8U);

    if(save_f){
        //ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
        RCLCPP_INFO(node_->get_logger(), "Writing map occupancy data to %s", mapdatafile.c_str());
        out = fopen(mapdatafile.c_str(), "w");
        if (!out)
        {
            //ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
            RCLCPP_ERROR(node_->get_logger(), "Couldn't save map file to %s", mapdatafile.c_str());
            return;
        }
        fprintf(out, "P5\n# CREATOR: Map_generator.cpp %.3f m/pix\n%d %d\n255\n",
                map.info.resolution, map.info.width, map.info.height);
    }
    for(unsigned int y = 0; y < map.info.height; y++) {
        for(unsigned int x = 0; x < map.info.width; x++) {
            // in ptr for map file
            unsigned int i = x + (map.info.height - y - 1) * map.info.width;
            // put ptr for mat_map_
            unsigned int i_mat;
            if(y_reverse_ == false){
                i_mat = i;
            }
            else{
                i_mat = x + y * map.info.width;
            }
            unsigned int i2;

            if (map.data[i] == 0) { //occ [0,0.1)
                //fputc(254, out);
                //fputc(255, out);    // 0xff  white
                if(save_f) fputc(FREE_AREA, out);
                mat_map_.data[i_mat] = FREE_AREA;
            } 
            //else if (map->data[i] == +100) { //occ (0.65,1]
            //    fputc(000, out);
            //} 
            //else { //occ [0.1,0.65]
            //    fputc(205, out);
            //}
            else if (map.data[i] < 0) {     // 未チェック領域
                //fputc(128, out);
                if(save_f) fputc(UNKNOWN_AREA, out);
                mat_map_.data[i_mat] = UNKNOWN_AREA;
            } 
            else{                       // 障害領域
                if(save_f) fputc(100-map.data[i],out);
                mat_map_.data[i_mat] = 100-map.data[i];
            }
        }
    }
    if(save_f) fclose(out);
    std::string mapmetadatafile = mapname_ + ".yaml";
    if(save_f){
        //ROS_INFO("Writing map occupancy data to %s", mapmetadatafile.c_str());
        RCLCPP_INFO(node_->get_logger(), "Writing map occupancy data to %s", mapmetadatafile.c_str());
        yaml_fp = fopen(mapmetadatafile.c_str(), "w");
    }

    /*
resolution: 0.100000
origin: [0.000000, 0.000000, 0.000000]
#
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
    */

    //geometry_msgs::Quaternion orientation = map.info.origin.orientation;
    // ROS2
    // geometry_msgs/msg/Quaternion
    geometry_msgs::msg::Quaternion orientation = map.info.origin.orientation;

    //tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
    tf2::Matrix3x3 mat(tf2::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));

    double yaw, pitch, roll;
    mat.getEulerYPR(yaw, pitch, roll);

    //yaml_.resolution=map->info.resolution;
    //yaml_.origin[0] = map->info.origin.position.x;
    //yaml_.origin[1] = map->info.origin.position.y;
    //yaml_.origin[2] = yaw;
    //yaml_.img_width = map->info.width;
    //yaml_.img_height = map->info.height;
    mapm_.resolution=map.info.resolution;
    mapm_.origin[0] = map.info.origin.position.x;
    mapm_.origin[1] = map.info.origin.position.y;
    mapm_.origin[2] = yaw;
    mapm_.width = map.info.width;
    mapm_.height = map.info.height;

    if(save_f){
        fprintf(yaml_fp, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
                mapdatafile.c_str(), map.info.resolution, map.info.origin.position.x, map.info.origin.position.y, yaw);

        fclose(yaml_fp);
    }
    //ROS_INFO("Done\n");
    //saved_map_ = true;
}

/*-----------------
* GetMap::saveMapRaw(const nav_msgs::msg::OccupancyGrid &map, bool reverse_f)
*  cost map の インフレーションを処理して保存する
*   nav_msgs::msg::OccupancyGrid &map
*   bool y_reverse  : y axis reverse
*       true -> map => cv::Mat 変換で、y軸を逆順にコピーする。 
*           見た目 Rviz2 と同じ。
*           y_map = y_cv::Mat が一致するので処理するには簡単。
*           map saver の通常の保存時の処理。
*       false -> map => cv::Mat 変換で、y軸をそのままコピーする。 
*           見た目 Rviz2 と、上下逆になる。
*           y_map = map_height - y_cv::Mat - 1 の関係になる。
*
* https://answers.ros.org/question/163801/how-to-correctly-convert-occupancygrid-format-message-to-image/
*
* http://docs.ros.org/en/lunar/api/nav_msgs/html/msg/OccupancyGrid.html
* 
# The map data, in row-major order, starting with (0,0).  Occupancy
# probabilities are in the range [0,100].  Unknown is -1.
*  int8[] data
-------------------*/
void GetMap::saveMapRaw(const nav_msgs::msg::OccupancyGrid &map, bool y_reverse, int inf_size){
    mat_map_raw_ = cv::Mat::zeros(map.info.height,map.info.width,CV_8UC1);
    unsigned int y2;
    for(unsigned int y = 0; y < map.info.height; y++) {
        if(y_reverse == true)
            y2 = map.info.height - 1 - y;  // y軸を逆順にコピー 見た目 Rviz2 と同じになる。
        else
            y2 = y;  // そのままコピー。見た目 Rviz2 と、上下逆になる。 

        for(unsigned int x = 0; x < map.info.width; x++) {
            unsigned int i = x + (y * map.info.width);
            unsigned int j = x + (y2 * map.info.width);
            mat_map_raw_.data[j] = FREE_AREA;
            //if(map.data[i]==0){
            //    mat_map_raw_.data[j] = FREE_AREA;
            //}
            // Unknown is -1
            //else if (map.data[i] < 0) {     // 未チェック領域
            //    //fputc(128, out);
            //    mat_map_raw_.data[j] = UNKNOWN_AREA;
            //}
            //else{                       // 障害領域
            //    mat_map_raw_.data[j] = 100-map.data[i];
            //}
            // 100 -> black
            //   0 -> white : 自由領域
            // 0 から 100 に近付くにつれて、障害度が大きい
            // 黒 -> グレー の グラデーションにしたい。
            //if(map.data[i] > 90){   // 障害領域
            //if(map.data[i] > 95){   // 障害領域
            //if(map.data[i] > 97){   // 障害領域
            if(map.data[i] > inf_size){   // 障害領域
                char dtx = 100-map.data[i];
                //mat_map_raw_.data[j] = dtx*6;
                mat_map_raw_.data[j] = dtx*40;
            }
        }
    }
    //cv::imshow("ml_mat_map_raw_", mat_map_raw_);
    //cv::waitKey(100);
}

/*-------------------------
* class GetMap
* check_collision()
*   float x: 基本座標[M]
*   float y: 基本座標[M]
*   float &ox:
*   float &oy:
*   int r : ロボットの半径[pixcel] 5 -> 0.25[M] 7 -> 0.35[M]
*   int func:
* 障害物から適切な距離を求める
*  func=0  default
*    障害物の2値画像を用いて、障害物の重心から離れる
*  func=1
*    非障害物 2値画像を用いて、非障害物の重心の方へ近づく
--------------------------*/
void GetMap::check_collision(float x,float y,float &ox,float &oy,int r,int func){
    ox=x;
    oy=y;

    cv::Point center_p; // 円の中心位置
    //int r =5;      // 円の半径  30 -> 1.5[M]      7 ->  0.35[M]  5->0.25[M]
    //if(func!=0){
    //    r = 7;
    //}

    cv::Mat result,result2,mask;

    std::cout << "check_collision() func:"<< func << " r:"<< r <<std::endl;

    //x0 = (x_g + 0.5) * (double)line_w_ * yaml.resolution + yaml.origin[0];
    //y0 = (y_g + 0.5) * (double)line_w_ * yaml.resolution + yaml.origin[1];

    // 基本座標を、Mat map 座標に変換 
    int px = (int)((x - mapm_.origin[0]) / mapm_.resolution);
    int py = (int)((y - mapm_.origin[1]) / mapm_.resolution);

    // add by nishi 2024.12.30
    if(y_reverse_==true){    // Map -> cv::Mat の時、y軸を逆転している。
        // cv::Mat の y 軸を逆転させる。
        py = mapm_.height - py - 1;
    }

    //ロボットの移動先を、マスクの中心にします。
    center_p.x = px;
    center_p.y = py;

    // Mask画像 を作成
    //mask = cv::Mat::zeros(mat_bin_map_.rows, mat_bin_map_.cols, CV_8UC1);
    mask = cv::Mat::zeros(mapm_.height, mapm_.width, CV_8UC1);

    // ロボットの移動先を中心にした、半径r の円を描きます。
    cv::circle(mask, center_p, r, cv::Scalar(255),-1);

    // 障害物から離れる処理
    if(func==0){
        // 障害物 2値化画像に 円のマスクを実施
        mat_bin_map_.copyTo(result2,mask);

        // 白色領域の面積(ピクセル数)を計算する
        int white_cnt = cv::countNonZero(result2);

        //#define CHECK_COLLISION_TEST
        #if defined(CHECK_COLLISION_TEST)
            cv::namedWindow("GetMap::check_collision", cv::WINDOW_NORMAL);
            cv::imshow("GetMap::check_collision", result2);
            int rc = cv::waitKey(1000);
            cv::destroyWindow("GetMap::check_collision");
        #endif

        // 障害物と接しています。
        if(white_cnt > 0){
            std::cout << " white_cnt=" << white_cnt << std::endl;

            // 4. 見つかった障害物の重心を求めます。
            cv::Moments m = cv::moments(result2,true);
            // 重心
            double x_g = m.m10 / m.m00;
            double y_g = m.m01 / m.m00;

            std::cout << " x_g=" << x_g <<" y_g="<< y_g << std::endl;

            // map 重心を、基本座標にします。
            float x_gb = ((float)x_g + 0.5) * mapm_.resolution + mapm_.origin[0];
            float y_gb = ((float)y_g + 0.5) * mapm_.resolution + mapm_.origin[1];
            x_gb = round_my<float>(x_gb,3);
            y_gb = round_my<float>(y_gb,3);

            x = round_my<float>(x,3);
            y = round_my<float>(y,3);

            // 5. 重心とロボットの移動先 を通る直線を求めて、その直線上を、重心から遠ざかる方向に、
            // ロボットの移動先を 5[cm]移動させます。

            // 5.1 (x_gb,y_gb) から、 (x,y) の角度[rad] を求める
            // 自分からの目的地の方角
            float off_target_x = x - x_gb;
            float off_target_y = y - y_gb;

            //同一地点
            if(fabsf(off_target_x) == 0.0 && fabsf(off_target_y) ==0.0){
                // ここは、将来、非障害物の2値化と Mask を取って、今度は、その重心方向に
                // ロボットの移動先を N[cm]移動させます。
                return;
            }

            float theta_r = std::atan2(off_target_y,off_target_x);   //  [ragian]

            float dx,dy;
            //if(white_cnt < 10){
            if(white_cnt < 3){  // changed by nishi 2024.2.10
                // 5.2  (x,y) から、(x_gb,y_gb) とは逆方向に 20[cm] ずらす。changed by nishi 2024.3.10
                dx = std::cos(theta_r) * 0.2;
                dy = std::sin(theta_r) * 0.2;
            }
            else{
                // 5.2  (x,y) から、(x_gb,y_gb) とは逆方向に 40[cm] ずらす。changed by nishi 2024.3.10
                dx = std::cos(theta_r) * 0.4;
                dy = std::sin(theta_r) * 0.4;
            }

            ox=x+dx;
            oy=y+dy;

            std::cout << " ox=" << ox <<" oy="<< oy << std::endl;
        }
    }
    // 非障害物へ近づく処理
    else{
        // 非障害物 2値化画像に 円のマスクを実施
        mat_bin_free_map_.copyTo(result2,mask);

        // 白色領域の面積(ピクセル数)を計算する
        int white_cnt = cv::countNonZero(result2);

        // 非障害物と接しています。
        if(white_cnt > 0){

            std::cout << " white_cnt=" << white_cnt << std::endl;

            // 4. 見つかった非障害物の重心を求めます。
            cv::Moments m = cv::moments(result2,true);
            // 重心
            double x_g = m.m10 / m.m00;
            double y_g = m.m01 / m.m00;

            std::cout << " x_g=" << x_g <<" y_g="<< y_g << std::endl;

            // map 重心を、基本座標にします。
            float x_gb = ((float)x_g + 0.5) * mapm_.resolution + mapm_.origin[0];
            float y_gb = ((float)y_g + 0.5) * mapm_.resolution + mapm_.origin[1];
            x_gb = round_my<float>(x_gb,3);
            y_gb = round_my<float>(y_gb,3);

            x = round_my<float>(x,3);
            y = round_my<float>(y,3);

            // 5. 重心とロボットの移動先 を通る直線を求めて、その直線上を、非障害物の重心に近づく方向に、
            // ロボットの移動先を 30-15[cm]移動させます。

            // 5.1 (x,y) から (x_gb,y_gb) の角度[rad] を求める
            // 自分からの目的地の方角
            float off_target_x = x_gb - x;
            float off_target_y = y_gb - y;

            //同一地点
            if(fabsf(off_target_x) == 0.0 && fabsf(off_target_y) ==0.0){
                // ここは、将来、障害物の2値化画像とのMask を取って、今度は、その重心方向から逆に
                // ロボットの移動先を N[cm]移動させます。
                return;
            }

            float theta_r = std::atan2(off_target_y,off_target_x);   //  [ragian]

            float dx,dy;
            if(white_cnt < 10){
                // 5.2  (x,y) から、(x_gb,y_gb) の方へ 40[cm] ずらす。
                dx = std::cos(theta_r) * 0.40;
                dy = std::sin(theta_r) * 0.40;
            }
            else{
                // 5.2  (x,y) から、(x_gb,y_gb) の方へ 30[cm] ずらす。
                dx = std::cos(theta_r) * 0.30;
                dy = std::sin(theta_r) * 0.30;
            }

            ox=x+dx;
            oy=y+dy;

            std::cout << " ox=" << ox <<" oy="<< oy << std::endl;
        }
    }
}
/*
* conv_fmt2(nav_msgs::OccupancyGrid_ map_msg)
*/
//void GetMap::conv_fmt2(boost::shared_ptr<const nav_msgs::OccupancyGrid_<std::allocator<void>>> map){
void GetMap::conv_fmt2(std::shared_ptr<const nav_msgs::msg::OccupancyGrid> map){
    
    //m_grid = np.array(map_msg.data).reshape((map_msg->info.height, map_msg->info.width));

    //std::cout << "m_grid.shape=" << m_grid.shape << std::endl;

    //print m_grid
    //np.place(m_grid,m_grid == -1, 255);    // replace all -1 to 255 
    //m_grid = m_grid.astype(np.uint8);

    // ここで、Rviz の画像に合わせてみる。
    if (_match_rviz == true){
        //m_grid = np.rot90(m_grid)   // 90度回転
        //m_grid = np.fliplr(m_grid)

        //a_trans = m_grid.transpose();
        //m_grid = a_trans;
        //m_grid = np.rot90(m_grid);   // 90度回転
        //m_grid = np.rot90(m_grid);   // 90度回転
    }
    if (false){
        //cv2.imwrite('xxx3.jpg', m_grid);
        //cv2.namedWindow("image",cv2.WINDOW_NORMAL);
        //cv2.imshow("image", m_grid);

        //cv2.waitKey(0);
        //cv2.destroyAllWindows();
    }

    //self.map_data = m_grid;

    uint32_t height = int(map->info.height / _line_w);
    uint32_t width = int(map->info.width / _line_w);

    std::cout << "map_msg.info.height,map_msg.info.width=" << map->info.height<<"," <<map->info.width << std::endl;
    std::cout << "height,width=" << height << "," << width << std::endl;

    //self.grid = cv2.resize(m_grid,(width,height)) // resize map_msg.data
    //self.grid = self.np_resize(m_grid,(width,height));s

    //np.place(self.grid,self.grid <= self.free_thresh ,0);
    //np.place(self.grid,self.grid > self.free_thresh ,255);

    //std::cout <<  "self.grid.shape=" << self.grid.shape << std::endl;

    if (false){
        //cv2.imwrite('xxx4.jpg', self.grid)
        //cv2.namedWindow("image",cv2.WINDOW_NORMAL)
        //cv2.imshow("image", self.grid)

        //cv2.waitKey(0)
        //cv2.destroyAllWindows()
    }
}

/*-------------------------
* class GetMap
* check_obstacle()
*  Map上の障害物を、x を中心にした、半径r の 円で、88度毎に判定する
*   float x: real world[M]
*   float y: real world[M]
*   float rz: ロボットの今の向き [radian]
*   float r_lng: 判定円の半径 [M]
*   int func=0  default
*
*  Rviz2 では、上が X 方向だが、右に90度回転させると、Real World と一致する。
*                    +y
*       1            |
*    2  x  0   -x ---+---> +x 軸 進行方向
*       3            |
*                    -y
* Map -> cv::mat だと、y軸逆か!!
* コールする前に、GetMap::Get() をコールしてください。
* 参考ページ
* http://cvwww.ee.ous.ac.jp/opencv_practice4/
*/
int GetMap::check_obstacle(float x,float y,float rz,float r_lng,int func,int black_thresh){

    int rc=0;
    cv::Point center_p; // 円の中心位置

    cv::Mat result,result2,mask;

    std::cout << "start GetMap::check_obstacle() func=" << func;

    // real world 座標を、Mat map 座標に変換
    int px = (int)((x - mapm_.origin[0]) / mapm_.resolution);
    int py = (int)((y - mapm_.origin[1]) / mapm_.resolution);

    // add by nishi 2024.12.30
    if(y_reverse_==true){    // Map -> cv::Mat の時、y軸を逆転している。
        // cv::Mat の y 軸を逆転させる。
        py = mapm_.height - py - 1;
    }

    //ロボットの現在位置を、マスクの中心にします。
    center_p.x = px;
    center_p.y = py;

    // Mask画像 を作成
    mask = cv::Mat::zeros(mapm_.height, mapm_.width, CV_8UC1);
    // 円弧、扇形を描く
	// ellipse(画像, 中心座標, Size(x径, y径), 楕円の回転角度, 始点角度, 終点角度, 色, 線幅, 連結)

    //int rr=12;
    //int rr=(int)(r_lng/0.05);
    // changed by nishi 2024.9.24
    int rr=(int)(r_lng/mapm_.resolution);

    // ロボットの今の向き[degree]
    int dz=(int)(rz*RADIANS_F);

    std::cout << " dz=" << dz << std::endl;

    // /home/nishi/usr/local/src/cpp-nishi/opencv-test1/main-12.cpp
    // 右が、0 下が 90 上が -90   -- 反時計回り
    switch(func){
        case 0:
            // func=0
            cv::ellipse(mask, center_p, cv::Size(rr, rr), 0, -44+dz, 44+dz, cv::Scalar(255), -1, cv::LINE_AA);
        break;
        case 3:
            // func=1 -> 3  cv_mat だと、y軸逆か!!
            cv::ellipse(mask, center_p, cv::Size(rr, rr), 0, -44+dz, -44-90+dz, cv::Scalar(255), -1, cv::LINE_AA);
        break;
        case 2:
            // func=2
            cv::ellipse(mask, center_p, cv::Size(rr, rr), 0, -44-90+dz, -44-90-90+dz, cv::Scalar(255), -1, cv::LINE_AA);
        break;
        case 1:
            // func=3 -> 1  cv_mat だと、y軸逆か!!
            cv::ellipse(mask, center_p, cv::Size(rr, rr), 0, 44+dz, 44+90+dz, cv::Scalar(255), -1, cv::LINE_AA);
        break;
    }
    //#define TEST_KK2
    #if defined(TEST_KK2)
        cv::imshow("check_o_msk", mask);
        cv::waitKey(0);
        cv::destroyAllWindows();
    #endif

    //#define TEST_KK2_A
    #if defined(TEST_KK2_A)
        cv::imshow("check_o_mat", mat_bin_map_);
        cv::waitKey(0);
        cv::destroyAllWindows();
    #endif

    // 障害物 2値化画像に 円のマスクを実施
    mat_bin_map_.copyTo(result2,mask);

    //#define TEST_KK2_A2
    #if defined(TEST_KK2_A2)
        cv::imshow("check_o_result2", result2);
        cv::waitKey(0);
        cv::destroyAllWindows();
    #endif

    // 黒色領域の面積(ピクセル数)を計算する
    int black_cnt = cv::countNonZero(result2);

    std::cout << " black_cnt=" << black_cnt;

    if(black_cnt <= black_thresh){
        black_cnt=0;
    }
    std::cout << " adjust black_cnt=" << black_cnt << std::endl;
    return black_cnt;
}

/*-------------------------
* class GetMap
* cource_obstacle_eye()
*  走行予定コース上の、Mapの障害物を、robo_radian_marker*2 幅で、チェックする。
*   float s_x
*   float s_y
*   float d_x
*   float d_y
*   float robo_radian_marker
*   int black_thresh
*   float check_range
*/
int GetMap::cource_obstacle_eye(float s_x,float s_y,float d_x,float d_y,float robo_radian_marker,int black_thresh,float check_range){

    cv::Mat result,result2,mask;

    std::cout << "GetMap::cource_obstacle_eye()" << std::endl;

    std::cout << " map_frame_:"<<map_frame_<< std::endl;
    std::cout << " map_orient_fix_:" << map_orient_fix_ << std::endl;

    int s_px,s_py,d_px,d_py,p_robo_radian;

    p_robo_radian = (int)(robo_radian_marker/mapm_.resolution);

    // 従来の処理
    // static map or cost map with global_frame: map
    if(map_orient_fix_==true){
        // real world 座標を、Mat map 座標に変換 
        s_px = (int)((s_x - mapm_.origin[0]) / mapm_.resolution);
        s_py = (int)((s_y - mapm_.origin[1]) / mapm_.resolution);

        d_px = (int)((d_x - mapm_.origin[0]) / mapm_.resolution);
        d_py = (int)((d_y - mapm_.origin[1]) / mapm_.resolution);
    }
    // cost map with global_frame: base_footprint
    // costmap の 中心が、ロボットの位置で、+x が、常にロボットの前方映像になる。
    else{
        // costmap cv:Mat の中心を、robo p0(0,0) とする。
        s_px = (int)((0.0 - mapm_.origin[0]) / mapm_.resolution);
        s_py = (int)((0.0 - mapm_.origin[1]) / mapm_.resolution);

        // robo 前方 +x 、0.4[M] をマスクにする。
        float x1 = check_range * std::cos(0.0);
        float y1 = check_range * std::sin(0.0);

        d_px = (int)((x1 - mapm_.origin[0]) / mapm_.resolution);
        d_py = (int)((y1 - mapm_.origin[1]) / mapm_.resolution);
    }
    // add by nishi 2024.12.30
    if(y_reverse_==true){    // Map -> cv::Mat の時、y軸を逆転している。
        // cv::Mat の y 軸を逆転させる。
        s_py = mapm_.height - s_py - 1;
        d_py = mapm_.height - d_py - 1;
    }

    //std::cout << " mapm_.origin[0]:" << mapm_.origin[0] << std::endl;
    //std::cout << " mapm_.origin[1]:" << mapm_.origin[1] << std::endl;
    //std::cout << " mapm_.origin[2]:" << mapm_.origin[2] << std::endl;

    // Mask画像 を作成
    mask = cv::Mat::zeros(mapm_.height, mapm_.width, CV_8UC1);

    // 直線を引く。
    cv::Point sp(s_px, s_py);	// 始点座標(x,y)
    cv::Point dp(d_px, d_py);	// 終点座標(x,y)

    // 両端を少し、扁平にしたいけど?
    cv::line(mask, sp, dp, cv::Scalar(255), p_robo_radian*2, cv::LINE_AA);

    //#define CHK_COURCE_OBSTACLE_TEST1
    #if defined(CHK_COURCE_OBSTACLE_TEST1)
        cv::imshow("eye_bin_map_", mat_bin_map_);
        cv::waitKey(100);
        //cv::destroyAllWindows();
    #endif

    //#define CHK_COURCE_OBSTACLE_TEST2
    #if defined(CHK_COURCE_OBSTACLE_TEST2)
        cv::imshow("eye_mask", mask);
        cv::waitKey(100);
        //cv::destroyAllWindows();
    #endif

    // 障害物 2値化画像に 円のマスクを実施
    mat_bin_map_.copyTo(result2,mask);

    #if defined(CHK_COURCE_OBSTACLE_TEST2)
        cv::imshow("eye_result2", result2);
        cv::waitKey(100);
        //cv::destroyAllWindows();
    #endif

    // 黒色領域の面積(ピクセル数)を計算する
    int black_cnt = cv::countNonZero(result2);

    std::cout << " black_cnt=" << black_cnt<< std::endl;
    if(black_cnt <= black_thresh){
        black_cnt=0;
    }
    std::cout << " adjust black_cnt=" << black_cnt << std::endl;
    return black_cnt;
}

/*
test_plot()
    float x,y: ロボット位置(基準座標) [M]
    float r_yaw: 基準座標での角度。 [rad]
    float robot_r=0.3 : ロボット半径
    ロボットの位置をマップにプロットする。
*/
void GetMap::test_plot(float x,float y,float r_yaw,float robot_r, std::string sub_title){

    std::cout << "GetMap::test_plot()" << std::endl;

    bool rc=get();
    if(rc != true){
        std::cout << " error end" << std::endl;
        return;
    }

    std::cout << " mapm_.origin[0]:"<< mapm_.origin[0]<< " mapm_.origin[1]:"<< mapm_.origin[1] << "mapm_.resolution:"<< mapm_.resolution  << std::endl;

    cv::Point center_p; // 円の中心位置
    cv::Mat my_map;

    // real world 座標を、Mat map 座標に変換 
    int px = (int)((x - mapm_.origin[0]) / mapm_.resolution);
    int py = (int)((y - mapm_.origin[1]) / mapm_.resolution);

    // add by nishi 2024.12.30
    if(y_reverse_==true){    // Map -> cv::Mat の時、y軸を逆転している。
        // cv::Mat の y 軸を逆転させる。
        py = mapm_.height - py - 1;
    }

    //ロボットの現在位置を、マスクの中心にします。
    center_p.x = px;
    center_p.y = py;

    //my_map=mat_map_.clone();
    my_map=mat_bin_map_.clone();
    //int rr=(robot_r/mapm_.resolution);
    int rr=(robot_r/0.05);

    rr=6;
    //rr=7;

    int dz=(int)(r_yaw*RADIANS_F);

    std::cout << " px:"<< px <<" py:"<< py << " rr:" << rr << std::endl;

    // 回転四角形
    // # RotatedRect(中心座標, サイズ(x, y), 回転角度degree)
    //cv::RotatedRect rect1(cv::Point2f( 80, 80), cv::Size(rr, rr), 0);
    cv::RotatedRect rect1(center_p, cv::Size(rr*2, rr*2), 0);

    // ロボットの外形(円)を描く
    // # ellipse(画像, RotatedRect, 色, 線幅, 連結)
    //cv::ellipse(my_map, rect1, cv::Scalar(255, 0, 255), 1, cv::LINE_AA);
    //cv::ellipse(my_map, rect1, cv::Scalar(255), 1, cv::LINE_AA);
    cv::ellipse(my_map, rect1, cv::Scalar(160), 1, cv::LINE_AA);

    //ロボットの向き。円弧を描く
    //cv::ellipse(my_map, center_p, cv::Size(rr, rr), 0, -30+dz, 30+dz, cv::Scalar(255), -1, cv::LINE_AA);
    cv::ellipse(my_map, center_p, cv::Size(rr, rr), 0, -30+dz, 30+dz, cv::Scalar(160), -1, cv::LINE_AA);

    // comment out by nishi 2024.8.31
    //cv::namedWindow("GetMap_plot"+sub_title, cv::WINDOW_NORMAL);

    cv::imshow("GetMap_plot"+sub_title, my_map);
    int key = cv::waitKey(1000);

    // comment out by nishi 2024.8.31
    //cv::destroyWindow("GetMap_plot"+sub_title);

}


#if defined(XXXX_X)
void GetMap::conv_dot2grid(self,x,y){
    xi= int(x / self.resolution);
    yi= int(y/ self.resolution);
    gx = int(xi / self.line_w);
    gy = int(yi / self.line_w);
    return gx,gy;
}
/*
convert Grid to Meter World
*/
void GetMap::conv_grid2meter(self,gx,gy,f_or_l){
    if self.match_rviz == True:
        return self.conv_grid2meter_m_p(gx,gy,f_or_l)
    else:
        return self.conv_grid2meter_m_m(gx,gy,f_or_l)
}
/*
Rviz と同じ図形 にした場合。
左上: M(+x,+y)    右上: M(-x,+y)
左下: M(+x,-y)    右下: M(-x,-y)
*/
void GetMap::conv_grid2meter_m_p(self,gx,gy,f_or_l){
    //x = (210 - (gx * self.line_w + self.line_w/2)) * self.resolution
    x =  -(gx * self.line_w + self.line_w/2) * self.resolution - self.org_x // chnaged by nishi 2020.12.2
    //y = (210 - (gy * self.line_w + self.line_w/2)) * self.resolution
    y =  -(gy * self.line_w + self.line_w/2) * self.resolution - self.org_y  // chnaged by nishi 2020.12.2
    return x,y
}
/*
Map データの図形 : y =  x の線対称の図形(x,y の入れ替え)
左上: M(-x,-y)    右上: M(-x,+y)
左下: M(+x,-y)    右下: M(+x,+y)
*/
void GetMap::conv_grid2meter_m_m(self,gx,gy,f_or_l){
    //x = (-210 + (gx * self.line_w + self.line_w/2)) * self.resolution
    x = (gx * self.line_w + self.line_w/2) * self.resolution + self.org_x   // chnaged by nishi 2020.12.2
    //y = (-210 + (gy * self.line_w + self.line_w/2)) * self.resolution
    y = (gy * self.line_w + self.line_w/2) * self.resolution + self.org_y   // chnaged by nishi 2020.12.2
    return y,x
}

void GetMap::get_rotation_matrix(self,rad){
    /*
    指定したradの回転行列を返す
    */
    rot = np.array([[np.cos(rad), -np.sin(rad)],
                    [np.sin(rad), np.cos(rad)]]);
    return rot;
}
void GetMap::np_resize(self,m_grid,req_size){
    (w,h) = req_size;
    (w0,h0) = m_grid.shape;
    size_x = int(w0 / w);
    size_y = int(h0 / h);
    o_dt = np.zeros((w, h),dtype='uint8');

    for i in range(w){
        for j in range(h){
            //av = np.mean(m_grid[i*size_x:(i+1)*size_x,j*size_y:(j+1)*size_y])
            av = np.max(m_grid[i*size_x:(i+1)*size_x,j*size_y:(j+1)*size_y])    // changed by nishi 2020.12.2
            //if av < 255:
            //    print 'av=',av
            if (math.isnan(av) == false){
                o_dt[i,j]= int(av)
                if o_dt[i,j] < 255:
                    print 'o_dt[',i,',',j,']=',o_dt[i,j];
            }
            else{
                print 'GetMap.np_resize() : resize error';
            }
        }
    }
    return o_dt;
}
#endif


/*-------------------------
* class Grid
--------------------------*/
//void Grid::init(nav_msgs::MapMetaData map_info,int line_w,std::vector<int8_t> data){
void Grid::init(nav_msgs::msg::MapMetaData map_info,int line_w,std::vector<int8_t> data){
    resolution_ = map_info.resolution;

    origin_x_ = map_info.origin.position.x;
    origin_y_ = map_info.origin.position.y;

    width_=map_info.width;
    height_=map_info.height;
    line_w_=line_w;
    size_x_ = width_ / line_w;
    size_y_ = height_ / line_w;

    //x0_ = origin_x_;
    //y0_ = origin_y_ - (double)height_ * (double)resolution_;


    std::cout << "x_size,y_size=" << size_x_ <<"," << size_y_ << std::endl;

    //blk_ = boost::shared_ptr<int8_t[]>( new int8_t( size_x_ *  size_y_) );
    //blk_ = std::shared_ptr<int8_t[]>( new int8_t( size_x_ *  size_y_) );
    if (blk_ == nullptr)
        blk_ = new int8_t[ size_x_ *  size_y_];


    if(blk_ == nullptr){
        std::cout << "alloc blk_ NG" << std::endl;
        return;
    }

    init_ok=true;
    std::cout << "alloc blk_ OK" << std::endl;

    updateGrid(data);

    std::cout << "init blk_ OK" << std::endl;

    saveGrid();

}

void Grid::updateGrid(std::vector<int8_t> data){
    int8_t d,d2;
    int di,cnt_i;
    int x_cur;
    int y_cur;
    int d_cur;

    if (init_ok != true){
        std::cout << "alloc blk_ not yet!" << std::endl;
        return;
    }
    //---------------------------------
    // http://docs.ros.org/en/lunar/api/nav_msgs/html/msg/OccupancyGrid.html
    //
    //# The map data, in row-major order, starting with (0,0).  Occupancy
    //# probabilities are in the range [0,100].  Unknown is -1.
    //---------------------------------

    // all raws
    for(int y=0;y < size_y_; y++){
        // all columuns of a raw 
        for(int x=0;x < size_x_; x++){
            d=-1;
            di=0;
            cnt_i=0;
            // set up 1 grid 
            for(int yy = 0; yy < line_w_; yy++){
                for(int xx=0; xx < line_w_; xx++){
                    y_cur = y*line_w_ + yy;
                    x_cur = x*line_w_ + xx;
                    if(y_cur < height_&& x_cur < width_){
                        d2 = data[y_cur * width_ + x_cur];
                        di += d2;
                        //if(d2 > d){
                        //    d=d2;
                        //}
                        cnt_i++;
                    }
                }
            }
            d_cur = y*size_x_ + x; 
            if(d_cur < size_y_*size_x_){
                //blk_[d_cur] = d;
                //di /= line_w_*line_w_;
                di /= cnt_i;
                blk_[d_cur] = (int8_t)di;
                if(di > 100){
                    std::cout << "di=" << di << std::endl;
                }
            }
            else{
                std::cout << "out of index of blk_" << std::endl;
            }
        }
    }

    std::cout << "update Grid OK" << std::endl;

}

void Grid::saveGrid(){
    std::string mapname_ ="/home/nishi/Grid_save";
    std::string mapdatafile = mapname_ + ".pgm";
    //ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
    FILE* out = fopen(mapdatafile.c_str(), "w");
    if (!out)
    {
        //ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
        return;
    }

    //fprintf(out, "P5\n# CREATOR: Map_generator.cpp %.3f m/pix\n%d %d\n255\n",
    //        map->info.resolution, map->info.width, map->info.height);

    fprintf(out, "P5\n# CREATOR: Map_generator.cpp %.3f m/pix\n%d %d\n255\n",
            0.05, size_x_, size_y_);

    for(unsigned int y = 0; y < size_y_; y++) {
        for(unsigned int x = 0; x < size_x_; x++) {
            unsigned int i = x + (size_y_ - y - 1) * size_x_;
            if (blk_[i] == 0) { //occ [0,0.1) 走行可領域
                //fputc(254, out);    // 0xfe   white
                fputc(255, out);    // 0xff  white
            } 
            else if (blk_[i] < 0) {     // 未チェック領域
                fputc(128, out);
            } 
            else{                       // 障害領域
                fputc(100-(char)blk_[i],out);
            }
        }
    }
    fclose(out);

    std::cout << "save Grid OK" << std::endl;

}

void Grid::gridToWorld(unsigned int gx, unsigned int gy, double& wx, double& wy)
{
  wx = origin_x_ + (((double)gx+0.5) * (double)line_w_) * resolution_;
  wy = origin_y_ + (((double)gy+0.5) * (double)line_w_) * resolution_;
}


//bool Grid::worldToGrid(double wx, double wy, unsigned int& gx, unsigned int& gy){
//    unsigned int ui_x = (unsigned int)((x0_ - wx) / resolution_);
//    unsigned int ui_y = (unsigned int)((y0_ - wy) / resolution_);
//    gx = ui_x / line_w_;
//    gy = ui_y / line_w_;
//}


bool Grid::worldToGrid(double wx, double wy, unsigned int& gx, unsigned int& gy)
{
  if (wx < origin_x_ || wy < origin_y_)
    return false;

  gx = (int)((wx - origin_x_) / resolution_);
  gy = (int)((wy - origin_y_) / resolution_);

  if (gx < size_x_ && gy < size_y_)
    return true;

  return false;
}


