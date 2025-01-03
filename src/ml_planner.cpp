/*
* Machine Learning Planner
*  turtlebot3_navi_my/src/ml_planner.cpp
*
*/

#include "turtlebot3_navi_my/ml_planner.hpp"

namespace ml_planner {


void MlPlanner::init(std::shared_ptr<rclcpp::Node> node,Robot_DriveCore *drive, GetMap *get_gcostmap, Path_plan *path_plan){

    std::cout << "MlPlanner::init()" << std::endl;

    node_=node;
    drive_=drive;
    get_gcostmap_ = get_gcostmap;

    path_plan_ = path_plan;

    // check data directory
    // https://cpprefjp.github.io/reference/filesystem/exists.html

    std::error_code ec;
    is_data_path_ok_ = std::filesystem::exists(data_path_, ec);

    std::cout << " is_data_path_ok_:"<< is_data_path_ok_ << std::endl;

    // add by nishi 2024.12.23
    #if defined(USE_OPP_TFLITE)
        std::cout << " opp_tflite init" << std::endl;
        settings_.model_path = "/home/nishi/Documents/VisualStudio-TF/opp_with_lstm/a.model.tflite";
        opp_tfl_.init(&settings_);
    #endif
}

/*
make_plann()
    走行コースの、障害物を判定して、経路計画を作る。
    dx,dy: roboto の、目的地 (tf real 空間)
    bool make_img:  画像を保存する/しない
*/
bool MlPlanner::make_plann(float dx,float dy,bool make_img,bool opp_on){

    std::cout << "MlPlanner::make_plann() called" << std::endl;

    drive_->get_tf(2);
    tf2::Vector3 start_origin = drive_->base_tf.getOrigin();

    float sx = start_origin.getX();
    float sy = start_origin.getY();

    return make_plann_(sx, sy, dx, dy, make_img, opp_on);
}

/*
make_plann_()
    走行コースの、障害物を判定して、経路計画を作る。
    sx,sy -> start : roboto の、現在位置 (tf real 空間)
    dx,dy -> end : roboto の、目的地 (tf real 空間)
    bool make_img:  画像を保存する/しない
*/
bool MlPlanner::make_plann_(float sx,float sy,float dx,float dy, bool make_img, bool opp_on){

    float off_x = dx - sx;
    float off_y = dy - sy;

    cv::Mat cropped;

    path_plan_->reset();

    // start - stop の距離を求める。
    float dist = std::sqrt((float)(off_x*off_x + off_y*off_y));

    if (dist < min_dist_ || dist > max_dist_){
        std::cout << " distance is too short or too long." << std::endl;
        return false;
    }

    // cost map のインフレーションを削った、mapデータを取得
    if(get_gcostmap_->get(false,true,y_reverse_,inf_size_) != true){
        std::cout << " MlPlanner::make_plann_() get_map error occured." << std::endl;
        return false;
    }

    // 従来の、 global costmap
    //#define SHOW_ML_MAT_MAP_
    #if defined(SHOW_ML_MAT_MAP_)
        cv::imshow("ml_mat_map_", get_gcostmap_->mat_map_);
        cv::waitKey(100);
    #endif

    std::cout << " get_gcostmap_->mat_map_raw_.cols:"<< get_gcostmap_->mat_map_raw_.cols << " get_gcostmap_->mat_map_raw_.rows:" << get_gcostmap_->mat_map_raw_.rows << std::endl;

    // 上下を、 rviz と同じになるように逆転させずに、インフレーションを削ったもの。 x2 のサイズ
    //#define SHOW_ML_MAT_MAP_RAW_
    #if defined(SHOW_ML_MAT_MAP_RAW_)
        cv::imshow("ml_mat_map_raw_", get_gcostmap_->mat_map_raw_);
        cv::waitKey(100);
        //cv::waitKey(0);
    #endif

    // start -> end の、ロボットの走行幅の矩形を抽出する。

    // https://qiita.com/kelbird/items/cc19a20840577da43dbe
    // cv::Mat roi_clone = src(cv::Rect(30, 30, 50, 50)).clone();

    // OpenCVで画像から回転矩形領域を切り出す
    // https://qiita.com/vs4sh/items/93d65468a992af5b8f92

    // tf real -> cv::mat postion に変換
    // worldToGrid(float wx, float wy,int& gx,int& gy,MapM& mapm)
    // y軸は、取り込む方法によって、逆転させる必要がある。
    int sx_m,sy_m,dx_m,dy_m;

    tf_world2MatMap(sx, sy,sx_m,sy_m,get_gcostmap_->mapm_,y_reverse_);
    //worldToGrid(sx, sy,sx_m,sy_m,get_gcostmap_->mapm_);
    // y axis が逆転している場合
    //if(y_reverse_==true)
    //    // cv::Mat の y 軸を逆転させる。
    //    sy_m = get_gcostmap_->mapm_.height - sy_m - 1;

    tf_world2MatMap(dx, dy,dx_m,dy_m,get_gcostmap_->mapm_,y_reverse_);
    //worldToGrid(dx, dy,dx_m,dy_m,get_gcostmap_->mapm_);
    // y axis が逆転している場合
    //if(y_reverse_==true)
    //    // cv::Mat の y 軸を逆転させる。
    //    dy_m = get_gcostmap_->mapm_.height - dy_m - 1;

    // 高さ half = ry_/ 0.05 -> 30[dot]
    float h_h = (ry_ / get_gcostmap_->mapm_.resolution);

    get_bound_rect(get_gcostmap_->mat_map_raw_, sx_m, sy_m, dx_m, dy_m,cropped, h_h);
    //get_bound_rect_old(get_gcostmap_->mat_map_raw_, sx_m, sy_m, dx_m, dy_m,cropped, h_h*2);

    cv::Mat cropped_resize;
    cv::resize(cropped, cropped_resize,cv::Size(), 2.0, 2.0);

    //#define SHOW_CROPPED_RESIZE
    #if defined(SHOW_CROPPED_RESIZE)
        cv::imshow("cropped_resize", cropped_resize);
        cv::waitKey(100);
    #endif

    // ml data 保存ディレクトリーがあります。
    if(is_data_path_ok_==true && make_img==true){
        // max_no_ 未初期化です。
        if(max_no_==-1){
            std::vector<std::string> file_names;
            getFileNames(data_path_, file_names);

            max_no_=0;
            int no;
            for(std::string s : file_names){
                std::cout << " s:"<< s << std::endl;
                if(s.length() > 6){
                    if(s.substr(s.size() - 6, 6) == "-l.jpg"){
                        continue;
                    }
                }
                if(s.length() > 4){
                    if(s.substr(s.size() - 4, 4) == ".jpg"){
                        s.erase(s.size() - 4, 4);
                        std::cout << " s:"<< s << std::endl;
                        try{
                            no = stoi(s);
                            if (no > max_no_)
                                max_no_=no;
                        }
                        catch(...){
                        }
                    }
                }
            }
        }
        max_no_++;
        std::cout << " max_no:"<< max_no_ << std::endl;
        // https://www.sejuku.net/blog/49199
        std::ostringstream oss;
        oss << max_no_ << ".jpg";

        // save to file
        // http://opencv.jp/opencv-2.1/cpp/reading_and_writing_images_and_video.html
        // bool imwrite(const string& filename, const Mat& img, const vector<int>& params=vector<int>())
        //cv::imwrite(data_path_+"/1.jpg",cropped_resize);
        cv::imwrite(data_path_+"/"+oss.str(),cropped_resize);
    }

    std::vector<u_int8_t> pred_y;
    int x_interval;
    int x_max;
    #define USE_LSTM
    #if defined(USE_LSTM)
        x_interval=2;
        x_max=300;
    #else
        x_interval=4;
        x_max=150;
    #endif

    // opp with lstm を行います。
    if(opp_on==true){
        path_plan_->init(sx, sy, off_x, off_y, y_reverse_, get_gcostmap_->mapm_.resolution, 0.025);

        cv::Mat predict_img=cropped_resize.clone();
        int width = predict_img.cols;

        opp_tfl_.predict(&cropped_resize, pred_y);
        // 結果を、プロットする。
        //int x=x_interval;
        int x=0;    // cur_x からの predict みたい。
        if(pred_y.size() > 0){
            for(int k=0;k<x_max;k++){
                //std::cout << soft_max_idx[k] <<std::endl;
                int idx= pred_y[k];
                int y=0;
                int y2=0;
                bool valid=false;
                if(x < width){
                    if (idx < 49){
                        valid=true;
                        if(y_reverse_==true)
                            y2 = (24 - idx)*2;  // y 軸 は、こちら!!
                        else
                            y2 = (idx - 24)*2;  // y 軸が逆みたい。
                        std::cout << std::to_string(idx) << ",";
                        y = 12 + idx*2;
                        cv::circle(predict_img, cv::Point(x,y), 2, cv::Scalar(128),-1);
                    }
                    path_plan_->append(x,y2,valid);
                    path_plan_->plan_done_=true;
                }
                x +=x_interval;
            }
            std::cout << std::endl;
        }
        else{
            std::cout << "predict nothing !!" << std::endl;
        }
        #define SHOW_PREDICT_IMG
        #if defined(SHOW_PREDICT_IMG)
            cv::imshow("predict_img", predict_img);
            cv::waitKey(100);
            //cv::waitKey(0);
        #endif

        if(path_plan_->is_all_valid() == true){
            // get_gcostmap_->mat_map_ に、プロットしてみる。
            //cv::Mat opp_plan_img=get_gcostmap_->mat_map_.clone();
            cv::Mat opp_plan_img = get_gcostmap_->mat_map_raw_.clone();

            for(Path_val dt : path_plan_->path_val_){
                float x_ar = dt.x_ar;
                float y_ar = dt.y_ar;

                std::cout << " x_ar:"<< x_ar <<" y_ar:"<< y_ar << std::endl;
                int sx_m,sy_m;
                tf_world2MatMap(x_ar, y_ar,sx_m,sy_m,get_gcostmap_->mapm_,y_reverse_);
                //worldToGrid(x_ar, y_ar,sx_m,sy_m,get_gcostmap_->mapm_);
                // y axis が逆転している場合
                //if(y_reverse_==true)
                //    sy_m = get_gcostmap_->mapm_.height - sy_m - 1;
                cv::circle(opp_plan_img, cv::Point(sx_m, sy_m), 1, cv::Scalar(128),-1);
                //std::cout << " sx_m:"<<sx_m <<" sy_m:"<< sy_m << std::endl;
            }
            #define SHOW_OPP_PLAN_IMG
            #if defined(SHOW_OPP_PLAN_IMG)
                cv::imshow("opp_plan_img", opp_plan_img);
                cv::waitKey(100);
                //cv::waitKey(0);
            #endif
        }
    }
    return true;
}

/*
get_bound_rect()
    robo 走行幅の、回転矩形を抽出する。
    cv::Mat &src : source cv::Mat
    sx_m,sy_m -> start : roboto の、現在位置 cv::mat address
    dx_m,dy_m -> end : roboto の、目的地 cv::mat address
    cv::Mat &cropped : output cv::Mat
    float h_h : cource width half

    1) ロボットのstart 位置を中心にして、画像全体を回転させる。
      回転角度は、 start - stop の直線と、 x軸の角度の逆を使う
    2) ロボット位置から +x 軸方向に、 走行幅 x 走行距離 の rectangle を抽出する。
    注) ロボット位置が、画像の端の時、回転方向が、画像の領域外になる時、画素が消えてしまうのではないか?
     抽出の画像を倍にする。

*/
void MlPlanner::get_bound_rect(cv::Mat &src,int sx_m,int sy_m,int dx_m, int dy_m, cv::Mat &cropped, float h_h){
    // arctan を求める
    // https://cpprefjp.github.io/reference/cmath/atan2.html
    int off_x = dx_m - sx_m;
    int off_y = dy_m - sy_m;

    // 目的地への角度
    float tan_rad = std::atan2((float)off_y, (float)off_x);

    // start - stop の距離を求める。
    float dist = std::sqrt((float)(off_x*off_x + off_y*off_y));

    // 画像の回転座標 -> ロボットの起点
    int cnter_x_m = sx_m;
    int cnter_y_m = sy_m;

    cv::Mat rotate;  // 回転画像
    //cv::Mat cropped; // 切り出された画像

    // https://docs.hsp.moe/OpenCV/453/cpp/ja/classcv_1_1_rotated_rect.html

    // 回転の中心位置 --> ロボットstart position
    cv::Point2f center_f((float)cnter_x_m,(float)cnter_y_m);

    // 回転角
    double degree = tan_rad * RADIANS_F;  // 回転角度  -> 正回転させる。
    double scale = 1.0; //大きさの定義

    // 2次元の回転行列を作成
    cv::Mat change = cv::getRotationMatrix2D(center_f, degree, scale); //回転&拡大縮小
    // cv::warpAffine(元画像, 変換後画像, 変換行列, 画像の大きさ, 補完方法, ピクセル外挿方法, 色); 
    cv::warpAffine(src, rotate, change, src.size()*2, cv::INTER_CUBIC,cv::BORDER_CONSTANT,cv::Scalar(0, 0, 0)); //画像の変換(アフィン変換)

    // 高さ = ry_
    //float h_h = (ry_ / get_gcostmap_->mapm_.resolution);
    //float h_h = 60;

    //#define ML_PLANNER_TEST11
    #if defined(ML_PLANNER_TEST11)
        // 選択された、矩形の確認
        //cv::rectangle(画像の変数, cv::Point(左上の点のx座標,左上の点のy座標), cv::Point(右下の点のx座標,右下の点のy座標), cv::Scalar(青,緑,赤), 線の太さ(-1の時は塗りつぶし));
        //左上(300,300),右下(500,400)に線の太さ5の四角形を描画
        cv::Point lt = cv::Point(sx_m, sy_m-h_h);
        cv::Point rb = cv::Point(sx_m+(int)dist, sy_m+h_h);

        //cv::rectangle(rotate, lt, rb, cv::Scalar(0,0,255), 1);
        cv::rectangle(rotate, lt, rb, cv::Scalar(128), 1);
        cv::imshow("rotate", rotate);
        cv::waitKey(100);
    #endif

    // 矩形抽出をする
    // 元画像がすでに回転しているので、
    // ロボット位置から、 +x軸上に切り出す。
    // 高さは、 center の 1[dot] 分を加える。  add by nishi 2024.10.12
    //cv::Size rect_size(dist,h_h*2);
    cv::Size rect_size(dist,h_h*2+1);
    cv::Point rect_center(sx_m+(int)dist/2, sy_m);

    // 回転した画像から矩形領域を切り出す．
    // http://opencv.jp/opencv-2svn/cpp/imgproc_geometric_image_transformations.html
    cv::getRectSubPix(rotate, rect_size, rect_center, cropped);

    //#define SHOW_CROPPED
    #if defined(SHOW_CROPPED)
        cv::imshow("cropped", cropped);
        cv::waitKey(100);
    #endif
}

/*
get_bound_rect_old()
    robo 走行幅の、回転矩形を抽出する。
    sx_m,sy_m -> start : roboto の、現在位置 cv::mat address
    dx_m,dy_m -> end : roboto の、目的地 cv::mat address
    bool make_img:  画像を保存する/しない
*/
void MlPlanner::get_bound_rect_old(cv::Mat &src,int sx_m,int sy_m,int dx_m,int dy_m,cv::Mat &cropped,float h_f){
    // arctan を求める
    // https://cpprefjp.github.io/reference/cmath/atan2.html
    int off_x = dx_m - sx_m;
    int off_y = dy_m - sy_m;

    float tan_rad = std::atan2((float)off_y, (float)off_x);

    // 走行幅矩形のCenter を求める。
    int cnter_x_m = (dx_m - sx_m)/2 + sx_m;
    int cnter_y_m = (dy_m - sy_m)/2 + sy_m;

    // https://docs.hsp.moe/OpenCV/453/cpp/ja/classcv_1_1_rotated_rect.html

    cv::Point2f center_f((float)cnter_x_m,(float)cnter_y_m);

    // start - stop の距離を求める。
    float dist = std::sqrt((float)(off_x*off_x + off_y*off_y));

    // 高さ = ry_
    //float h_f = (ry_ / get_gcostmap_->mapm_.resolution)*2;
    //float h_f = 60;

    cv::Size2f size_f(dist,h_f);
    cv::RotatedRect rect(center_f, size_f, tan_rad * RADIANS_F); // 回転矩形
    cv::Rect rect_val =	rect.boundingRect();

    //auto rect_val =	rect.boundingRect();
    //printf("%s",rect_val);

    std::cout << rect_val << std::endl;

    cv::Mat M; // 回転行列
    cv::Mat rotated; // 回転された元画像
    //cv::Mat cropped; // 切り出された画像

    float angle = rect.angle;
    cv::Size rect_size = rect.size;
    if (rect.angle < -45.) {
        angle += 90.0;
        std::swap(rect_size.width, rect_size.height);
    }

    // 回転矩形の角度から回転行列を計算する．
    M = cv::getRotationMatrix2D(rect.center, angle, 1.0);
    // 元画像を回転させる．
    cv::warpAffine(src, rotated, M, src.size(), cv::INTER_CUBIC);

    // 回転した画像から矩形領域を切り出す．
    // http://opencv.jp/opencv-2svn/cpp/imgproc_geometric_image_transformations.html
    cv::getRectSubPix(rotated, rect_size, rect.center, cropped);
    //cv::imshow("cropped", cropped);
    //cv::waitKey(100);
}

}