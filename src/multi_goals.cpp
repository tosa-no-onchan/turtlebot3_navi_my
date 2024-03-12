/*
* turtlebot3_navi_my/src/multi_goale.cpp
* https://docs.ros2.org/latest/api/rclcpp/classrclcpp_1_1Node.html
* https://answers.ros.org/question/307370/ros2-whats-the-best-way-to-wait-for-a-new-message/
* https://github.com/ros-planning/navigation2/blob/main/nav2_map_server/src/map_saver/map_saver.cpp
*/

#include "turtlebot3_navi_my/multi_goals.hpp"
//#include <cmath.h>

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
#ifdef USE_CHECK_COLL
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

void gridToWorld(int gx, int gy, float& wx, float& wy,MapM& mapm)
{
    //wx = ((float)gx + 0.5) * yaml.resolution + yaml.origin[0];
    //wy = ((float)(yaml.img_height-gy) + 0.5) * yaml.resolution + yaml.origin[1];

    wx = ((float)gx + 0.5) * mapm.resolution + mapm.origin[0];
    wy = ((float)gy + 0.5) * mapm.resolution + mapm.origin[1];
}

bool worldToGrid(float wx, float wy,int& gx,int& gy,MapM& mapm)
{
    if (wx < mapm.origin[0] || wy < mapm.origin[1])
        return false;
    gx = (int)((wx - mapm.origin[0]) / mapm.resolution);
    gy = (int)((wy - mapm.origin[1]) / mapm.resolution);

    if (gx < mapm.width && gy < mapm.height)
        return true;
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
* class AnchorFinder
* init()
* blk_ の初期化をします。
* 1blk : 50[cm] x 50[cm]  -> 1[dot] = 50[cm]
--------------------------*/
void AnchorFinder::init(){
    blk_mapm_.resolution = 0.25;         // 1dot = 0.25[M]
    blk_mapm_.origin[0] = border_def.bot_l.x;        // x = -10.525
    blk_mapm_.origin[1] = border_def.bot_l.y;        // y = -10.525
    blk_mapm_.origin[2] = 0.0;                      // yaw
    int blk_size = int(blk_mapm_.resolution*100.0);               // 1[dot] = 25[cm]
    float w = (border_def.top_r.x + std::fabs(border_def.bot_l.x)) / blk_mapm_.resolution;    // 42.1
    float h = (border_def.top_r.y + std::fabs(border_def.bot_l.y)) / blk_mapm_.resolution;    // 42.1
    //float ceil(float x); 
    //blk_w_ = (int)ceil(w);    // 85[dot]
    //blk_h_ = (int)ceil(h);    // 85[dot]
    blk_mapm_.width = (int)ceil(w);      // 85[dot]
    blk_mapm_.height = (int)ceil(h);      // 85[dot]

    //std::cout << "blk_h_=" << blk_h_ << std::endl;
    //std::cout << "blk_w_=" << blk_w_ << std::endl;

    //blk_ = cv::Mat::zeros(y,x,CV_8U);
    blk_ = cv::Mat::zeros(blk_mapm_.height,blk_mapm_.width,CV_8U);     //  85 x 85
    blk_f_=true;

    std::cout << "AnchorFinder::init() img_width="  << blk_mapm_.width << " ,img_height=" << blk_mapm_.height << std::endl;

    //std::exit(0);
}
/*-------------------------
* class AnchorFinder
* check_Border()
--------------------------*/
bool AnchorFinder::check_Border(float x,float y){
    if(x > border_def.top_r.x || x < border_def.bot_l.x)
        return false;
    if(y > border_def.top_r.y || y < border_def.bot_l.y)
        return false;
    return true;
}
/*-------------------------
* class AnchorFinder
* mark_blk_world(float x,float y)
* float wx: 基本座標 X
* float wy: 基本座標 Y
* blk_ に、使用済をセット
--------------------------*/
void AnchorFinder::mark_blk_world(float wx,float wy){
    int bx,by;

    //worldToBlock(wx, wy,bx,by);
    worldToGrid(wx, wy, bx, by,blk_mapm_);

    int blk_w_;      // blk_  width
    int blk_h_;      // blk_ height

    if(blk_w_ == 0 || blk_h_==0){
        std::cout << "mark_blk_world() error #1 blk_w_="  << blk_w_ << " ,blk_h_=" << blk_h_ << std::endl;
        //std::exit(0);
    }
    if(bx > blk_w_ || by > blk_h_){
        std::cout << "mark_blk_world() error #2 wx="  << wx << " ,wy=" << wy << std::endl;
        std::cout << " blk_w_="  << blk_w_ << " ,blk_h_=" << blk_h_ << std::endl;
        std::cout << " bx="  << bx << " ,by=" << by << std::endl;
        //while(1){ }
    }
    else{
        blk_.data[by * blk_w_ +bx]=255;
    }
}
/*-------------------------
* class AnchorFinder
* mark_blk(int bx,int by)
* int bx: Block X
* int by: Block Y
* blk_ に、使用済をセット
--------------------------*/
void AnchorFinder::mark_blk(int bx,int by,u_int8_t mark){
    if(blk_mapm_.width == 0 || blk_mapm_.height == 0){
        std::cout << "mark_blk() error #1 blk_mapm_.width="  << blk_mapm_.width << " ,blk_mapm_.height=" << blk_mapm_.height << std::endl;
        std::cout << "call  std::exit(0)" << std::endl;
        std::exit(0);
    }

    if(bx > blk_mapm_.width || by > blk_mapm_.height){
        std::cout << "mark_blk_world() error #2" << std::endl;
        std::cout << " blk_mapm_.width="  << blk_mapm_.width << " ,blk_h_=" << blk_mapm_.height << std::endl;
        std::cout << " bx="  << bx << " ,by=" << by << std::endl;
    }
    else{
        blk_.data[by * blk_mapm_.width +bx]=mark;
    }
}

/*-------------------------
* class AnchorFinder
* sort_blob()
--------------------------*/
void AnchorFinder::sort_blob(float cur_x,float cur_y){

    std::vector<Gpoint> *g_ponts_ptr=nullptr;
    //std::list<Gpoint> *g_ponts_ptr=nullptr;

    switch(block_mode){
        case 0:
            g_ponts_ptr = &g_points_;
        break;
        case 1:
            g_ponts_ptr = &g_points1;
        break;
        case 2:
            g_ponts_ptr = &g_points2;
        break;
    }
    for(int i=0;i < g_ponts_ptr->size();i++){
        float off_x = g_ponts_ptr->at(i).x - cur_x;
        float off_y = g_ponts_ptr->at(i).y - cur_y;

        g_ponts_ptr->at(i).dist = std::sqrt(off_x*off_x+off_y*off_y);

    }
    // dist でソート 大きい順
    switch(block_mode){
        case 0:
            std::sort(g_points_.begin(),g_points_.end(),compare_Gpoint_dist_max);
        break;
        case 1:
            std::sort(g_points1.begin(),g_points1.end(),compare_Gpoint_dist_max);
        break;
        case 2:
            std::sort(g_points2.begin(),g_points2.end(),compare_Gpoint_dist_max);
        break;
    }
}

/*-------------------------
* class AnchorFinder
* check()
* GetMap* getmap
* float cur_x : ロボットの位置 World point X (基本座標)
* float cur_y : ロボットの位置 World point Y (基本座標)
--------------------------*/
#if defined(USE_FUTURE_GET_MAP)
void AnchorFinder::check(GetMap* getmap,float cur_x,float cur_y){

    cv::Mat bin_img;
    //cv::namedWindow("gry", cv::WINDOW_NORMAL);
    //cv::namedWindow("thres", cv::WINDOW_NORMAL);
    //cv::namedWindow("reverse", cv::WINDOW_NORMAL);

    getmap_=getmap;

    //mapm_=mapm;             // map Yaml
    mapm_=getmap->mapm_;             // map Yaml

    //grid_mapm_=mapm;       // Grid Yaml
    grid_mapm_=getmap->mapm_;       // Grid Yaml
    grid_mapm_.resolution=mapm_.resolution * (float)line_w_;     // 0.25  1[dot] -> 0.25[M]

    if(view_f_m){
        //cv::namedWindow("img", cv::WINDOW_AUTOSIZE);
        cv::namedWindow("img", cv::WINDOW_NORMAL);
        cv::namedWindow("blob", cv::WINDOW_NORMAL);

    }
    if(view_f){
        //cv::namedWindow("thres", cv::WINDOW_AUTOSIZE);
        //cv::namedWindow("thres", cv::WINDOW_NORMAL);
        cv::namedWindow("unknown", cv::WINDOW_NORMAL);
        cv::namedWindow("block", cv::WINDOW_NORMAL);
        //cv::namedWindow("label", cv::WINDOW_NORMAL);
    }

    //rgb = cv::imread("aaa.png");
    //cv::cvtColor(rgb, gry, cv::COLOR_BGR2GRAY);
    //gry = cv::imread("/home/nishi/map_builder.pgm",0);
    //gry = cv::imread("../map_builder-house.pgm",0);
    //gry = cv::imread("../map_builder-gass.pgm",0);
    //gry = cv::imread("../Grid_save.pgm",0);


    //cv::imshow("gry", gry);


    //cv::waitKey(0);
    //return 0;

    //2. 障害物を、2値画像 and 反転します。
	//int thresh = 10;        // 障害物の値 0-100 / 未知領域:128  / 自由領域:255
	//threshold(gry, img_dst, thresh, 255, cv::THRESH_BINARY);
    // 障害物 < 10 だけを、白にします。
	//cv::threshold(mat_map, bin_img, thresh, 255, cv::THRESH_BINARY_INV);
    // soft copy
    bin_img=getmap->mat_bin_map_;

    //cv::imshow("BinReverse", bin_img);

    //--------------------
    // 3. ここから、上記、2値画像 を、ロボットの回転円の直径の Grid に分割します。
    // 障害物のセルが1個でもあれば、その Grid を障害物とします。
    //--------------------
    
    //int   line_w_ = 5;  // ラインの幅 -> grid size [dot]  0.05[m] * 5 = 25[cm]
    //int   line_w_ = 4;  // ラインの幅 -> grid size [dot]  0.05[m] * 4 = 20[cm]

    int8_t d;
    int x_cur;
    int y_cur;
    int d_cur;

    u_int32_t height_ = bin_img.rows;       // 421[dot]
    u_int32_t width_ = bin_img.cols;        // 421[dot]

    size_x_ = width_ / line_w_;
    size_y_ = height_ / line_w_;

    if(width_ % line_w_)
        size_x_++;
    if(height_ % line_w_)     
        size_y_++;

    cv::Mat block = cv::Mat::zeros(size_y_,size_x_,CV_8U);
    grid_mapm_.width = size_x_;
    grid_mapm_.height = size_y_;

    // all raws
    for(int y=0;y < size_y_; y++){
        // all columuns of a raw 
        for(int x=0;x < size_x_; x++){
            d=0;
            //di=0;
            //cnt_i=0;
            // set up 1 grid
            for(int yy = 0; yy < line_w_ && d==0; yy++){
                for(int xx=0; xx < line_w_ && d==0; xx++){
                    y_cur = y*line_w_ + yy;
                    x_cur = x*line_w_ + xx;
                    if(y_cur < height_&& x_cur < width_){
                        // 障害物(白セル)です?
                        if(bin_img.data[y_cur * width_ + x_cur] == 0xff){
                            d++;
                        }
                    }
                }
            }
            // 半分は、障害物(白セル)です。
            if(d >= line_w_*line_w_/13){
                block.data[y*size_x_ + x] = 0xff;
            }
        }
    }

    //cv::imshow("block", block);

    //----------------------
    // 4. 上でマークされたブロックをブロブ分割(ラベリング)します。
    // 今回は、Labeling.h を使います。
    // https://imura-lab.org/products/labeling/
    //-----------------------

    LabelingBS	labeling;

    int nlabel =0;  // 使用済ラベルの数(次割当ラベル番号)
    //int w = img.cols;
    int w = size_x_;
    //int h = img.rows;
    int h = size_y_;

    //cv::Mat mat_label = cv::Mat::zeros(h,w,CV_8U);
    //cv::Mat mat_label2 = cv::Mat::zeros(h,w,CV_8U);

	//cv::Mat mat_blob;

	cv::Mat img_lab(h,w, CV_16SC1);

    //short *result = new short[ w * h ];

    labeling.Exec( block.data, (short *)img_lab.data, w, h, true, 0 );

    point_n_ = labeling.GetNumOfResultRegions();   // ラベル総数

    std::cout << "point_n_=" << point_n_ << std::endl;

    //------------------------
    // 5. アンカーを全て求めてみます。
    //-----------------------
    Gpoint g_point;
    if(g_points_.empty() != true){
        g_points_.clear();
    }
    g_points_.reserve(G_POINTS_MAX);


    RegionInfoBS	*ri;
    float x_g,  y_g;

    std::cout <<"display all g center start" << std::endl;

    cv::Mat mat_blob2;

    anchor_=0;
 
    for(int i=0;i<point_n_;i++){
        ri = labeling.GetResultRegionInfo( i );
        ri->GetCenter(x_g,y_g);   // 重心を得る

        std::cout <<"x_g="<< x_g << " , y_g=" << y_g << std::endl;

    	cv::compare(img_lab, i+1, mat_blob2, cv::CMP_EQ); // ラベル番号1 を抽出

        // 該当ブロブの周囲アンカーを得る
        anchoring(mat_blob2,cur_x,cur_y);
    }

    if(block_mode==0){
        // g_points_ が当初予定件数を超えています。
        if(g_points_.size() >= G_POINTS_MAX){
            std::cout <<"g_points_ execced G_POINTS_MAX size()=" << g_points_.size() << std::endl;
            std::cout <<"take condense g_points_" << std::endl;
            // g_points1 をコンデンスします。
            condense_Gpoint(&g_points_);
        }
        // dist でソート 大きい順
        std::sort(g_points_.begin(),g_points_.end(),compare_Gpoint_dist_max);
    }
    else{
        //std::vector<Gpoint> pg_wk1;
        //std::vector<Gpoint> pg_wk2;

        for(int i = 0;i < g_points_.size();i++){
            Gpoint gp = g_points_[i];
            // ボーダーチェック
            if(check_Border(gp.x,gp.y)!=true){
                std::cout <<"border check NG"<< std::endl;
                continue;
            }
            // ブラックポイントチェック
            if(find_Gpoint(gp.x,gp.y,g_points_black)==true){
                std::cout <<"g_points_black fined"<< std::endl;
                continue;
            }
            // block1 へ割り振り
            if(gp.x < block_line_x){
                std::cout <<"g_points1.push_back(g_points_["<<i<<"]) "<< std::endl;
                g_points1.push_back(gp);
            }
            // block2 へ割り振り
            else{
                std::cout <<"check g_points2 same x,y "<< std::endl;
                if(find_Gpoint(gp.x,gp.y,g_points2) == false){
                    std::cout <<"g_points2.push_back(g_points_["<<i<<"]) "<< std::endl;
                    g_points2.push_back(gp);
                }
            }
        }
        if(block_mode==1){
            std::cout << "sort(g_points1)" << std::endl;
            std::cout << "g_points1.size()=" << g_points1.size() << std::endl;
            // g_points1 が当初予定件数を超えています。
            if(g_points1.size() >= G_POINTS_MAX1_2){
                std::cout <<"g_points1.size() execced G_POINTS_MAX1_2 size=" << g_points1.size() << std::endl;
                std::cout <<"take condense g_points1" << std::endl;
                // g_points1 をコンデンスします。
                condense_Gpoint(&g_points1);
            }
            // dist でソート 大きい順
            std::sort(g_points1.begin(),g_points1.end(),compare_Gpoint_dist_max);

            std::cout << "end sort(g_points1)" << std::endl;
        }
        else{
            std::cout <<"sort(g_points2)"<< std::endl;
            std::cout <<"g_points2.size()="<< g_points2.size() << std::endl;
            // g_points2 が当初予定件数を超えています。
            if(g_points2.size() >= G_POINTS_MAX1_2){
                std::cout <<"g_points2.size() execced G_POINTS_MAX1_2 size=" << g_points2.size() << std::endl;
                std::cout <<"take condense g_points2" << std::endl;
                // g_points2 をコンデンスします。
                condense_Gpoint(&g_points2);
            }
            std::sort(g_points2.begin(),g_points2.end(),compare_Gpoint_dist_max);

            std::cout << "end sort(g_points2)" << std::endl;
        }
    }
}
#endif


/*-------------------------
* class AnchorFinder
* anchoring()
* float cur_x : ロボットの位置 World point X (基本座標)
* float cur_y : ロボットの位置 World point Y (基本座標)
--------------------------*/
void AnchorFinder::anchoring(cv::Mat &mat_blob2,float cur_x,float cur_y){

    //------------------------
    // 6. 対象ブロブについて、その外周上に、一定間隔でアンカーを置きます。
    //-----------------------

    //cv::Mat mat_anchor=mat_blob2.clone();

    // 『OpenCVによる画像処理入門』(講談社) P157 周囲長 を求める
    // 始点の探索
    int height=mat_blob2.rows;
    int width=mat_blob2.cols;
    int ini_x,ini_y;
    bool ok_f=false;
    for(int y=0; y<height && ok_f==false;y++){
        for(int x=0; x < width;x++){
            if(mat_blob2.data[y*width + x]==255){
                ini_y=y;
                ini_x=x;
                ok_f=true;
                break;
            }
        }
    }
    // 4近傍 下右上左
    int rot_x[4] = {0, 1, 0, -1};
    int rot_y[4] = {1, 0, -1, 0};
    int rot=0; // 探索方向
    int perrimeter = 0; // 周囲長
    int now_x,now_y;

    int pre_x=ini_x;
    int pre_y=ini_y;


    // 印を付ける  -> グレーの X 印
    //mat_anchor.data[((int)pre_y-1)*width+(int)pre_x-1] = 128;
    //mat_anchor.data[((int)pre_y-1)*width+(int)pre_x+1] = 128;
    //mat_anchor.data[(int)pre_y*width+(int)pre_x] = 128;
    //mat_anchor.data[((int)pre_y+1)*width+(int)pre_x-1] = 128;
    //mat_anchor.data[((int)pre_y+1)*width+(int)pre_x+1] = 128;

    // 今の場所を アンカーとして、g_points_ と blk_ へ登録する。
    anchor_put(pre_x, pre_y, cur_x, cur_y);

    while(1){
        for(int i=0; i<4;i++){
            now_x = pre_x + rot_x[(rot+i)%4];
            now_y = pre_y + rot_y[(rot+i)%4];
            if(now_x < 0 || now_x > width-1 || now_y < 0 || now_y > height-1 ){
                continue;
            }
            if(mat_blob2.data[now_y * width + now_x] == 255){
                pre_x = now_x;
                pre_y = now_y;
                perrimeter++;
                // 8 Grid 毎に 処理
                //if((perrimeter%10)==0){
                if((perrimeter%8)==0){
                    // 今の場所を アンカーとして、g_points_ と blk_ へ登録する。
                    anchor_put(pre_x, pre_y, cur_x, cur_y);
                }
                rot += i+3;
                break;
            }
        }
        if(pre_x == ini_x && pre_y == ini_y){
            break;
        }
    }
    //cv::imshow("anchor", mat_anchor);
}

/*-------------------------
* class AnchorFinder
* anchor_put()
*  int gx: grid x
*  int gy: grid y
*  float cur_x : ロボットの位置 World point X (基本座標)
*  float cur_y : ロボットの位置 World point Y (基本座標)
--------------------------*/
bool AnchorFinder::anchor_put(int gx,int gy,float cur_x,float cur_y){
    Gpoint g_point;
    float wx0,wy0,wx,wy;
    float off_x,off_y;

    // Grid to World(wx0,wy0)
    gridToWorld(gx, gy, wx0, wy0,grid_mapm_);

    // wx0,wy0 の 障害物チェック
    // 非障害物へ近づく補正
    #if defined(USE_FUTURE_GET_MAP)
        getmap_->check_collision(wx0,wy0,wx,wy,robo_r_,1);  // changed by nishi 2024.3.1
    #endif

    // アンカーの場所が、未処理のブロックかチェック
    int bx,by;
    // World to Block
    if(worldToGrid(wx, wy, bx, by,blk_mapm_)==false){
        std::cout <<"anchor is out of range" << std::endl;
        return false;
    }
    if(blk_.data[by*blk_mapm_.width + bx] == 0){

        // Block アドレスをセット
        g_point.xi=bx;
        g_point.yi=by;

        // World アドレスをセット
        g_point.x = round_my<float>(wx,2);
        g_point.y = round_my<float>(wy,2);

        off_x = wx - cur_x;     // 基本座標 の ロボットとの距離 dX
        off_y = wy - cur_y;     // 基本座標 の ロボットとの距離 dY

        // Distance をセット
        g_point.dist = std::sqrt(off_x*off_x+off_y*off_y);

        anchor_++;
        std::cout <<"anchor_="<< anchor_ << std::endl;
        g_points_.push_back(g_point);
    }
    return true;
}

/*-------------------------
* class AnchorFinder
* save_blk()
* blk_ を、画像で保存します。
--------------------------*/
void AnchorFinder::save_blk(){
    cv::imwrite("/home/nishi/AnchorFinder_blk_.pgm", blk_);
}

/*-------------------------
* class BlobFinder
* find_Gpoint()
--------------------------*/
bool BlobFinder::check_Border(float x,float y){
    if(x > border_def.top_r.x || x < border_def.bot_l.x)
        return false;
    if(y > border_def.top_r.y || y < border_def.bot_l.y)
        return false;
    return true;
}
/*-------------------------
* class BlobFinder
* sort_blob()
--------------------------*/
void BlobFinder::sort_blob(float cur_x,float cur_y){

    std::vector<Gpoint> *g_ponts_ptr=nullptr;
    //std::list<Gpoint> *g_ponts_ptr=nullptr;

    switch(block_mode){
        case 0:
            g_ponts_ptr = &g_points_;
        break;
        case 1:
            g_ponts_ptr = &g_points1;
        break;
        case 2:
            g_ponts_ptr = &g_points2;
        break;
    }
    for(int i=0;i < g_ponts_ptr->size();i++){
        float off_x = g_ponts_ptr->at(i).x - cur_x;
        float off_y = g_ponts_ptr->at(i).y - cur_y;

        g_ponts_ptr->at(i).dist = std::sqrt(off_x*off_x+off_y*off_y);

    }
    // dist でソート 大きい順
    switch(block_mode){
        case 0:
            std::sort(g_points_.begin(),g_points_.end(),compare_Gpoint_dist_max);
        break;
        case 1:
            std::sort(g_points1.begin(),g_points1.end(),compare_Gpoint_dist_max);
        break;
        case 2:
            std::sort(g_points2.begin(),g_points2.end(),compare_Gpoint_dist_max);
        break;
    }
}

/*-------------------------
* class BlobFinder
* check()
--------------------------*/
#if defined(USE_FUTURE_GET_MAP)
void BlobFinder::check(cv::Mat mat_map,MapM &mapm,float cur_x,float cur_y){
    cv::Mat rgb, gry, thres,reverse,neg,dst2;
    //cv::namedWindow("gry", cv::WINDOW_NORMAL);
    //cv::namedWindow("thres", cv::WINDOW_NORMAL);
    //cv::namedWindow("reverse", cv::WINDOW_NORMAL);


    if(view_f_m){
        //cv::namedWindow("img", cv::WINDOW_AUTOSIZE);
        cv::namedWindow("img", cv::WINDOW_NORMAL);
        cv::namedWindow("blob", cv::WINDOW_NORMAL);

    }
    if(view_f){
        //cv::namedWindow("thres", cv::WINDOW_AUTOSIZE);
        //cv::namedWindow("thres", cv::WINDOW_NORMAL);
        cv::namedWindow("unknown", cv::WINDOW_NORMAL);
        cv::namedWindow("block", cv::WINDOW_NORMAL);
        //cv::namedWindow("label", cv::WINDOW_NORMAL);
    }


    //rgb = cv::imread("aaa.png");
    //cv::cvtColor(rgb, gry, cv::COLOR_BGR2GRAY);
    //gry = cv::imread("/home/nishi/map_builder.pgm",0);
    //gry = cv::imread("../map_builder-house.pgm",0);
    //gry = cv::imread("../map_builder-gass.pgm",0);
    //gry = cv::imread("../Grid_save.pgm",0);


    //cv::imshow("gry", gry);


    //cv::waitKey(0);
    //return 0;


    int thresh=100;
    //int thresh=130;
    //int thresh=250;

    //cv::threshold(gry,thres,thresh,255,cv::THRESH_BINARY);

    //cv::imshow("thres", thres);


    // 白黒反転
    //thres.convertTo(reverse,thres.type(),-1.0,255.0);
    //cv::imshow("reverse", reverse);

    cv::Mat img=mat_map;
    
    //cv::Mat img = (cv::Mat_<u_int8_t>(6,6) << 
    //            0, 128, 0, 128, 128, 0,
    //            0, 255, 0, 255, 255, 0,
    //            128, 255, 255, 255, 0, 0,
    //            128, 255, 0, 0, 0, 0, 
    //            0,255, 0, 255, 255, 0,
    //            0, 0, 0, 128, 128, 0
    //            );  

    std::cout << "img.rows=" << img.rows << std::endl;
    std::cout << "img.cols=" << img.cols << std::endl;
    std::cout << "img.type()=" << img.type() << std::endl;
    std::cout << "img.step=" << img.step << std::endl;

    if(view_f_m){
        cv::imshow("img", img);
        //cv::waitKey(0);
        //return;
    }

    //cv::Mat dst = cv::Mat::zeros(img.rows,img.cols,CV_16U);
    cv::Mat unknown = cv::Mat::zeros(img.rows,img.cols,CV_8U);

    std::cout << "unknown.rows=" << unknown.rows << std::endl;
    std::cout << "unknown.cols=" << unknown.cols << std::endl;
    std::cout << "unknown.channels()=" << unknown.channels() << std::endl;
    std::cout << "unknown.type()=" << unknown.type() << std::endl;
    std::cout << "unknown.step=" << unknown.step << std::endl;
    

    //int thresh=100;
    //int thresh=130;
    //int thresh=250;

    //cv::threshold(img,thres,thresh,255,cv::THRESH_BINARY);

    //cv::imshow("thres", thres);

    //std::cout << (int)thres.at<u_int8_t>(0,0) << "," << (int)thres.at<u_int8_t>(0,1) << "," << (int)thres.at<u_int8_t>(0,2) << std::endl;
    //std::cout << (int)thres.at<u_int8_t>(1,0) << "," << (int)thres.at<u_int8_t>(1,1) << "," << (int)thres.at<u_int8_t>(1,2) << std::endl;
    //std::cout << (int)thres.at<u_int8_t>(2,0) << "," << (int)thres.at<u_int8_t>(2,1) << "," << (int)thres.at<u_int8_t>(2,2) << std::endl;


    //int nlabel =0;  // ラベルの数
    int w = img.cols;
    int h = img.rows;

    //const int TABLESIZE = 1024;
    //static int table[TABLESIZE];        // label 使用番号 管理テーブル
    //table[0]=0;

    int unknown_t[img.rows*img.cols];
    for(int y=0;y<img.rows;y++){
        for(int x=0;x < img.cols;x++){
            unknown_t[y*img.cols + x] = 0;
        }
    }
 

    //----------------
    // 1. 4近傍に 未知領域(-1) のある、自由領域(白) をピックアップして、unknown_t[]へ入れる
    //----------------
    for(int y = 0 ; y < h; y++){
        for (int x=0;x < w; x++){
            //std::cout << "y=" << y << ",x=" << x << std::endl;
            // 注目画素は、自由領域 でない(黒)
            if(img.data[y*w+x] != FREE_AREA){
               unknown_t[y * w + x] = 0;
            }
            // 注目画素は、自由領域(白 : 0xff) だ!!
            else{
                // 近傍8画素をチェック
                //const int N = 8;
                //const int dy[N] = { 1, 1,  1,  0, 0, -1, -1, -1};
                //const int dx[N] = {-1, 0,  1, -1, 1, -1 , 0 , 1};

                // 近傍4画素をチェック 上、下、左、右
                const int N = 4;
                const int dy[N] = { 1,  0, 0, -1};
                const int dx[N] = { 0, -1, 1,  0};

                int list[N];
                int count =0;
                for(int k = 0; k < N; k++){
                    int xdx = x + dx[k];
                    int ydy = y + dy[k];
                    int m = ydy * w + xdx;
                    // はみ出していない and  Unknown -1(0x80) セル
                    if(xdx >= 0 && ydy >= 0 && xdx < w && ydy < h && img.data[m] == UNKNOWN_AREA){
                        unknown_t[y*w+x] = 0xff;
                        unknown.data[y*w+x] = 0xff;
                        break;
                    }
                }
            }
        }
    }

    if(view_f){
        cv::imshow("unknown", unknown);
        //cv::waitKey(0);
        //return;
    }
    //--------------------
    // 2. ここから、上記、unknown_t を、ロボットの回転円の直径のブロックに分割します。
    // 未知領域(-1=0x80)に隣接する自由領域セル(0=0xff) が1個でもあれば、そのブロックをマーキングします。
    //--------------------

    //int   line_w_ = 5;  // ラインの幅 -> grid size [dot]  0.05[m] * 5 = 25[cm]

    int8_t d;
    int x_cur;
    int y_cur;
    int d_cur;

    u_int32_t height_ = img.rows;
    u_int32_t width_ = img.cols;

    int size_x_ = width_ / line_w_;
    int size_y_ = height_ / line_w_;

    if(width_ % line_w_)
        size_x_++;
    if(height_ % line_w_)     
        size_y_++;

    cv::Mat block = cv::Mat::zeros(size_y_,size_x_,CV_8U);

    // all raws
    for(int y=0;y < size_y_; y++){
        // all columuns of a raw 
        for(int x=0;x < size_x_; x++){
            d=0;
            //di=0;
            //cnt_i=0;
            // set up 1 grid
            for(int yy = 0; yy < line_w_ && d==0; yy++){
                for(int xx=0; xx < line_w_ && d==0; xx++){
                    y_cur = y*line_w_ + yy;
                    x_cur = x*line_w_ + xx;
                    if(y_cur < height_&& x_cur < width_){
                        // unknown セルに接している?
                        if(unknown_t[y_cur * width_ + x_cur] == 0xff){
                            d=0xff;
                        }
                    }
                }
            }
            if(d!=0){
                block.data[y*size_x_ + x] = 0xff;
            }
        }
    }

    if(view_f){
        cv::imshow("block", block);
        //cv::waitKey(0);
        //return;
    }

    //----------------------
    // 3. 上でマークされたブロックをブロブ分割(ラベリング)します。
    // 今回は、Labeling.h を使います。
    // https://imura-lab.org/products/labeling/
    //-----------------------

    int nlabel =0;  // 使用済ラベルの数(次割当ラベル番号)
    //int w = img.cols;
    w = size_x_;
    //int h = img.rows;
    h = size_y_;

    //cv::Mat mat_label = cv::Mat::zeros(h,w,CV_8U);
    //cv::Mat mat_label2 = cv::Mat::zeros(h,w,CV_8U);

	cv::Mat mat_blob;
	//cv::Mat img_lab(h,w, CV_16SC1);
    img_lab_ = cv::Mat::zeros(h,w,CV_16SC1);

    LabelingBS	labeling;

    labeling.Exec( block.data, (short *)img_lab_.data, w, h, true, 1 );

    point_n_ = labeling.GetNumOfResultRegions();   // ラベル総数
    std::cout << "point_n_=" << point_n_ << std::endl;

    //----------------------
    //4. ラベリングされたブロブで大きなブロブを
    // 1つ選んで、それの重心を求めて、その場所を、ロボットの移動場所とします。
    //----------------------

    RegionInfoBS	*ri;
    ri = labeling.GetResultRegionInfo( 0 );

	cv::compare(img_lab_, 1, mat_blob, cv::CMP_EQ); // ラベル番号1 を抽出

    float x_g,  y_g;
    ri->GetCenter(x_g,y_g);   // 重心を得る

	std::cout <<"w="<< w << ", h=" << h << std::endl;

    //ここが、ロボットの移動先
    // 実際使うには、 x_g,y_g を、標準座標に変換しないといけない。
	std::cout <<"x_g="<< x_g << " , y_g=" << y_g << std::endl;

    // blob の重心は、左上が、原点です
    abs_x_ = (x_g + 0.5) * (double)line_w_ * mapm.resolution + mapm.origin[0];
    abs_y_ = (y_g + 0.5) * (double)line_w_ * mapm.resolution + mapm.origin[1];
    yaw_ = mapm.origin[3];

	std::cout <<"mapm.resolution="<< mapm.resolution << std::endl;

	std::cout <<"abs_x_="<< abs_x_ << " , abs_y_=" << abs_y_ << std::endl;


    // 印を付ける  -> グレーの X 印
    mat_blob.data[((int)y_g-1)*w+(int)x_g-1] = 128;
    mat_blob.data[((int)y_g-1)*w+(int)x_g+1] = 128;
    mat_blob.data[(int)y_g*w+(int)x_g] = 128;
    mat_blob.data[((int)y_g+1)*w+(int)x_g-1] = 128;
    mat_blob.data[((int)y_g+1)*w+(int)x_g+1] = 128;
  
    if(view_f_m){
        cv::imshow("blob", mat_blob);
        cv::waitKey(0);
    }

    //------------------------
    // 5. 重心を全て求めてみます。
    //-----------------------
    std::cout <<"display all g center start" << std::endl;

    Gpoint g_point;
    if(g_points_.empty() != true){
        g_points_.clear();
    }
    g_points_.reserve(point_n_);

    for(int i=0;i<point_n_;i++){
        ri = labeling.GetResultRegionInfo( i );
        ri->GetCenter(x_g,y_g);   // 重心を得る

        float x0,y0;
        x0 = (x_g + 0.5) * (double)line_w_ * mapm.resolution + mapm.origin[0];
        y0 = (y_g + 0.5) * (double)line_w_ * mapm.resolution + mapm.origin[1];

        g_point.x = round_my<float>(x0,2);
        g_point.y = round_my<float>(y0,2);

        float off_x = x0 - cur_x;
        float off_y = y0 - cur_y;

        g_point.dist = std::sqrt(off_x*off_x+off_y*off_y);

        g_point.pic = ri->GetNumOfPixels();

        g_points_.push_back(g_point);

        std::cout <<"x_g="<< g_point.x << " , y_g=" << g_point.y << std::endl;
    }

    if(block_mode==0){
        // dist でソート 大きい順
        std::sort(g_points_.begin(),g_points_.end(),compare_Gpoint_dist_max);
    }
    else{
        //std::vector<Gpoint> pg_wk1;
        //std::vector<Gpoint> pg_wk2;

        for(int i = 0;i < g_points_.size();i++){
            Gpoint gp = g_points_[i];
            // ボーダーチェック
            if(check_Border(gp.x,gp.y)!=true){
                std::cout <<"border check NG"<< std::endl;
                continue;
            }
            // ブラックポイントチェック
            if(find_Gpoint(gp.x,gp.y,g_points_black)==true){
                std::cout <<"g_points_black fined"<< std::endl;
                continue;
            }
            // block1 へ割り振り
            if(gp.x < block_line_x){
                std::cout <<"g_points1.push_back(g_points_["<<i<<"]) "<< std::endl;
                g_points1.push_back(gp);
            }
            // block2 へ割り振り
            else{
                std::cout <<"check g_points2 same x,y "<< std::endl;
                if(find_Gpoint(gp.x,gp.y,g_points2) == false){
                    std::cout <<"g_points2.push_back(g_points_["<<i<<"]) "<< std::endl;
                    g_points2.push_back(gp);
                }
            }
        }

        if(block_mode==1){
            std::cout << "sort(g_points1)" << std::endl;
            std::cout << "g_points1.size()=" << g_points1.size() << std::endl;
            // dist でソート 大きい順
            //condense_Gpoint(&g_points1);
            std::sort(g_points1.begin(),g_points1.end(),compare_Gpoint_dist_max);

            std::cout << "end sort(g_points1)" << std::endl;
        }
        else{
            std::cout <<"sort(g_points2)"<< std::endl;
            std::cout <<"g_points2.size()="<< g_points2.size() << std::endl;
            //condense_Gpoint(&g_points2);
            std::sort(g_points2.begin(),g_points2.end(),compare_Gpoint_dist_max);

            std::cout << "end sort(g_points2)" << std::endl;
        }
    }
    return;
}
#endif


/*
* class MultiGoals
*/
//void MultiGoals::init(ros::NodeHandle &nh){
void MultiGoals::init(std::shared_ptr<rclcpp::Node> node){

    std::cout << "MultiGoals::init():#1 " << std::endl;

    //nh_=nh;
    node_=node;

	get_map_func_ = node_->declare_parameter<int>("get_map_func", get_map_func_);

    //navi.init(nh,2);
    //drive.init(nh,true);

    //drive.init(node);                // changed by nishi 2024.2.27
    #if defined(USE_MOVE_BASE)
        //drive_=&drive_nav; 
        // Sub1* sub1 = dynamic_cast<Sub1*>(new Base());  // ダウンキャスト
        //drive_=dynamic_cast<Robot_DriveCore *>(new RobotDriveNAV2());     
        drive_=dynamic_cast<Robot_DriveCore *>(&drive_nav);     

        drive_nav.init(node,&getTF,true);   
        drive_cmd.init(node_,&getTF,true);                // changed by nishi 2024.2.27
        mode_f_origin=mode_f=1;    // 1: navigation2 mode
    #else
        //drive_=&drive_cmd;
        //drive_=dynamic_cast<Robot_DriveCore *>(new RobotDriveCmd_Vel());     
        drive_=dynamic_cast<Robot_DriveCore *>(&drive_cmd);     
        drive_cmd.init(node_,&getTF,true);                // changed by nishi 2024.2.27
        mode_f_origin=mode_f=0;       // 0:vmd_vel mode
    #endif



    std::cout << "MultiGoals::init():#1.1 mode_f="<< mode_f << std::endl;

    #if defined(USE_FUTURE_GET_MAP)
        get_map.init(node,get_map_func_);
    #endif

    std::cout << "MultiGoals::init():#2 " << std::endl;


    goalId = 0;
    sts=0;
    t_type=0;

    force_start_origin=false;

    start_pos_x = start_pos_y = start_pos_z = 0.0;
    start_rot_rz = 0.0;  // add by nishi 2022.11.15 by nishi

    sleep(1);
}

/*
AutoMap I
auto_map()
    Auto Map builder
*/
void MultiGoals::auto_map(){
    std::cout << "Start Auto Map" << std::endl;

    float off;

    std::vector<Gpoint> *g_ponts_ptr=nullptr;

    if(blobFinder_.g_points1.empty() != true){
        blobFinder_.g_points1.clear();
    }
    if(blobFinder_.g_points2.empty() != true){
        blobFinder_.g_points2.clear();
    }
    if(blobFinder_.g_points_black.empty() != true){
        blobFinder_.g_points_black.clear();
    }
    blobFinder_.g_points1.reserve(50);
    blobFinder_.g_points2.reserve(50);
    blobFinder_.g_points_black.reserve(50);

    for(int lc=0; lc < 100 ;lc++){
        off=0.0;
        #if defined(USE_FUTURE_GET_MAP)
            get_map.get();
        #endif

        std::cout << "auto_map() #6 call drive.get_tf(2)" << std::endl;
        //drive.get_tf(2);      // changed by nishi 2024.2.28
        drive_->get_tf(2);

        //tf2::Vector3 cur_origin = drive.base_tf.getOrigin();
        tf2::Vector3 cur_origin = drive_->base_tf.getOrigin();    // changed by nishi 2024.2.28

        float cur_x = cur_origin.getX();
        float cur_y = cur_origin.getY();

        std::cout << "auto_map() #7 call blobFinder_.check()" << std::endl;
        // Find Next Unkonown Blob
        #if defined(USE_FUTURE_GET_MAP)
            blobFinder_.check(get_map.mat_map_,get_map.mapm_,cur_x,cur_y);
        #endif

        switch(blobFinder_.block_mode){
            case 0:
                g_ponts_ptr = &(blobFinder_.g_points_);
            break;
            case 1:
                g_ponts_ptr = &(blobFinder_.g_points1);
            break;
            case 2:
                g_ponts_ptr = &(blobFinder_.g_points2);
            break;
        }
        if(g_ponts_ptr->size() == 0){
            if(blobFinder_.block_mode >= 2){
                std::cout << "Auto Map next point nothing" << std::endl;
                return;
            }
            blobFinder_.block_mode++;
            std::cout << "Auto Map block_mode="<< blobFinder_.block_mode << std::endl;
            continue;
        }

        float dist, r_yaw,r_yaw_off;

        int j = g_ponts_ptr->size();
        int l=0;
        // 今回の Map 上の、blob を全て回ります。
        while(j > 0){
            j--;
            if(g_ponts_ptr->at(j).dist > 0.3){

                float ox,oy;

                // Collision Ahead - Exiting Spin spin failed が生じるの、組み込みました。将来、改善されれば、不要です。
                // ここで、ロボットの四方の障害物をチェックして、障害物から少しだけ、離れる。add by nishi 2024.3.7
                obstacle_escape();

                // 此処で、走査先の障害物との距離をチェック
                // ここの距離に余裕が必要では?  by nishi 2024.3.1
                #if defined(USE_FUTURE_GET_MAP)
                    //get_map.check_collision(g_ponts_ptr->at(j).x,g_ponts_ptr->at(j).y,ox,oy);
                    get_map.check_collision(g_ponts_ptr->at(j).x,g_ponts_ptr->at(j).y,ox,oy,blobFinder_.robo_r_);   // changed by nishi 2024.3.1
                #endif

                //drive.comp_dad(g_ponts_ptr->at(j).x,g_ponts_ptr->at(j).y,dist, r_yaw, r_yaw_off);
                //drive.comp_dad(ox,oy,dist, r_yaw, r_yaw_off);
                drive_->comp_dad(ox,oy,dist, r_yaw, r_yaw_off); // changed by nishi 2024.2.28

                // Navi move
                std::cout << ">> auto_map() Next, goes to x="<< ox+off << " ,y="<< oy+off << std::endl;

                #define USE_TEST_PLOT
                #if defined(USE_TEST_PLOT)
                    // plot してみる for Debug
                    get_map.test_plot(ox+off,oy+off,r_yaw,r_yaw_off);
                #endif

                //drive.navi_move(blobFinder_.abs_x_+off,blobFinder_.abs_y_+off,r_yaw);
                //if(drive.navi_move(g_ponts_ptr->at(j).x+off,g_ponts_ptr->at(j).y+off,r_yaw,r_yaw_off)==false){
                //if(drive.navi_move(ox+off,oy+off,r_yaw,r_yaw_off)==false){
                if(drive_->navi_move(ox+off,oy+off,r_yaw,r_yaw_off)==false){    // changed by nishi 2024.2.28
                    std::cout << ">> derive.navi_move() error end"<< std::endl;
                    std::cout << ">> black point appeend"<< std::endl;
                    blobFinder_.g_points_black.push_back(g_ponts_ptr->at(j));
                }
                std::cout << ">> Auto Map inner loop "<< l << " end"<< std::endl;
            }
            else{
                std::cout << ">> Auto Map inner loop "<< l << " pass"<< std::endl;
            }
            //末尾を削除
            g_ponts_ptr->pop_back();

            // ロボットの現在位置
            //cur_origin = drive.base_tf.getOrigin();
            cur_origin = drive_->base_tf.getOrigin();   // changed by nishi 2024.2.28

            cur_x = cur_origin.getX();
            cur_y = cur_origin.getY();
            // ブロブを、ロボットの現在位置の近い順(降順)にする
            blobFinder_.sort_blob(cur_x,cur_y);
            j = g_ponts_ptr->size();
            l++;
        }
        if(lc==0){
            blobFinder_.block_mode=1;
        }
        g_ponts_ptr->clear();

        std::cout << ">> Auto Map >>> lc="<< lc << std::endl;

        float d_yaw=90.0;
        // 360度回転
        //for(int k=0;k<4 ;k++){
        //   drive.rotate_off(d_yaw);
        //}

        // ここで、GetMap::check_obstacle() で、前方の障害物をチェックして、問題があれば、後ろを向かせる。add by nishi 2024.3.7
        obstacle_escape();

        // ここを、cmd_vel モードにしたらどう?  by nishi 2024.3.1
        set_drive_mode(0);      // set cmd_vel mode
        // 前方、60 をチェック
        drive_->rotate_off(30.0);   // changed by nishi 2024.2.28
        drive_->rotate_off(-60.0);  // changed by nishi 2024.2.28
        drive_->rotate_off(30.0);   // changed by nishi 2024.2.28
        set_drive_mode(1);      // set nav2 mode

    }
    std::cout << ">> End Auto Map" << std::endl;
}


/*
AutoMap II
auto_map_anchor
    Auto Map builder of Anchor

*/
void MultiGoals::auto_map_anchor(){

    std::cout << "Start Auto Map Anchor" << std::endl;

    float off;

    std::vector<Gpoint> *g_ponts_ptr=nullptr;

    if(anchorFinder_.g_points1.empty() != true){
        anchorFinder_.g_points1.clear();
    }
    if(anchorFinder_.g_points2.empty() != true){
        anchorFinder_.g_points2.clear();
    }
    if(blobFinder_.g_points_black.empty() != true){
        anchorFinder_.g_points_black.clear();
    }
    anchorFinder_.g_points1.reserve(G_POINTS_MAX1_2);
    anchorFinder_.g_points2.reserve(G_POINTS_MAX1_2);
    anchorFinder_.g_points_black.reserve(G_POINTS_MAX1_2);

    // AnchorFinder::blk_ の初期化 
    anchorFinder_.init();

    // 今は、アンカー位置とロボットの近い順に走査をしている。
    // では無くて、アンカーのコリジョンチェック後の位置とロボットの近い順にすべきかでは無いか?
    for(int lc=0; lc < 100 ;lc++){
        off=0.0;

        #if defined(USE_FUTURE_GET_MAP)
            get_map.get();
        #endif

        std::cout << "auto_map_anchor() #1 call drive.get_tf(2)" << std::endl;
        //drive.get_tf(2);
        drive_->get_tf(2);      // changed by nishi 2024.2.28

        //tf::Vector3 cur_origin = drive.base_tf.getOrigin();
        //tf2::Vector3 cur_origin = drive.base_tf.getOrigin();
        tf2::Vector3 cur_origin = drive_->base_tf.getOrigin();  // changed by nishi 2024.2.28

        float cur_x = cur_origin.getX();    // World point(基本座標)
        float cur_y = cur_origin.getY();    // World point(基本座標)

        std::cout << "auto_map_anchor() #2 call anchorFinder_.check()" << std::endl;
        // Find Next All Anchors
        #if defined(USE_FUTURE_GET_MAP)
            anchorFinder_.check(&get_map,cur_x,cur_y);
        #endif

        switch(anchorFinder_.block_mode){
            case 0:
                g_ponts_ptr = &(anchorFinder_.g_points_);
            break;
            case 1:
                g_ponts_ptr = &(anchorFinder_.g_points1);
            break;
            case 2:
                g_ponts_ptr = &(anchorFinder_.g_points2);
            break;
        }
        if(g_ponts_ptr->size() == 0){
            if(anchorFinder_.block_mode >= 2){
                std::cout << "Auto Map Anchor next point nothing" << std::endl;
                return;
            }
            anchorFinder_.block_mode++;
            std::cout << "Auto Map Anchor block_mode="<< anchorFinder_.block_mode << std::endl;
            continue;
        }

        float dist, r_yaw,r_yaw_off;

        int j = g_ponts_ptr->size();
        int l=0;
        // 今回の Map 上の、anchors を全て回ります。
        while(j > 0){
            j--;
            u_int8_t mark=255;
            if(g_ponts_ptr->at(j).dist > 0.25){

                // Collision Ahead - Exiting Spin spin failed が生じるの、組み込みました。将来、改善されれば、不要です。
                // ここで、ロボットの四方の障害物をチェックして、障害物から少しだけ、離れる。add by nishi 2024.3.7
                obstacle_escape();

                //float ox,oy;
                // 此処で、走査先の非障害物との距離をチェック
                //get_map.check_collision(g_ponts_ptr->at(j).x,g_ponts_ptr->at(j).y,ox,oy,7,1);

                //drive.comp_dad(g_ponts_ptr->at(j).x,g_ponts_ptr->at(j).y,dist, r_yaw, r_yaw_off);
                drive_->comp_dad(g_ponts_ptr->at(j).x,g_ponts_ptr->at(j).y,dist, r_yaw, r_yaw_off); // changed by nishi 2024.2.28

                // Navi move
                //drive.navi_move(blobFinder_.abs_x_+off,blobFinder_.abs_y_+off,r_yaw);
                std::cout << ">> auto_map_anchor() Next, goes to x="<< g_ponts_ptr->at(j).x+off << " ,y="<< g_ponts_ptr->at(j).y+off << std::endl;

                #define USE_TEST_PLOT2
                #if defined(USE_TEST_PLOT2)
                    // plot してみる for Debug
                    get_map.test_plot(g_ponts_ptr->at(j).x+off,g_ponts_ptr->at(j).y+off,r_yaw,r_yaw_off);
                #endif

                //if(drive.navi_move(g_ponts_ptr->at(j).x+off,g_ponts_ptr->at(j).y+off,r_yaw,r_yaw_off)==false){
                if(drive_->navi_move(g_ponts_ptr->at(j).x+off,g_ponts_ptr->at(j).y+off,r_yaw,r_yaw_off)==false){    // changed by nishi 2024.2.28
                    std::cout << ">> derive.navi_move() error end"<< std::endl;
                    //std::cout << ">> black point appeend"<< std::endl;
                    //anchorFinder_.g_points_black.push_back(g_ponts_ptr->at(j));
                    mark=128;
                }
                std::cout << ">> Auto Map Anchor inner loop "<< l << " end"<< std::endl;
            }
            else{
                std::cout << ">> Auto Map Anchor inner loop "<< l << " pass"<< std::endl;
            }
            // 走査済を設定
            anchorFinder_.mark_blk(g_ponts_ptr->at(j).xi,g_ponts_ptr->at(j).yi,mark);

            //末尾を削除
            g_ponts_ptr->pop_back();

            // ロボットの現在位置
            //cur_origin = drive.base_tf.getOrigin();
            cur_origin = drive_->base_tf.getOrigin();   // changed by nishi 2024.2.28

            cur_x = cur_origin.getX();
            cur_y = cur_origin.getY();
            // ブロブを、ロボットの現在位置の近い順(降順)にする
            anchorFinder_.sort_blob(cur_x,cur_y);
            j = g_ponts_ptr->size();
            l++;
        }
        if(lc==0){
            anchorFinder_.block_mode=1;
        }
        g_ponts_ptr->clear();

        std::cout << ">> Auto Map Anchor >>> lc="<< lc << std::endl;

        // AnchorFinder::blk_ を、画像に保存します。
        anchorFinder_.save_blk();

        float d_yaw=90.0;
        // 360度回転
        //for(int k=0;k<4 ;k++){
        //   drive.rotate_off(d_yaw);
        //}

        // ここで、GetMap::check_obstacle() で、前方の障害物をチェックして、問題があれば、後ろを向かせる。add by nishi 2024.3.7
        obstacle_escape();

        // ここを、cmd_vel モードにしたらどう?  by nishi 2024.3.1
        // 前方、60 をチェック
        set_drive_mode(0);  // set cmd_vel mode
        //drive.rotate_off(30.0);
        drive_->rotate_off(30.0);   // changed by nishi 2024.2.28
        //drive.rotate_off(-60.0);
        drive_->rotate_off(-60.0);  // changed by nishi 2024.2.28
        //drive.rotate_off(30.0);
        drive_->rotate_off(30.0);   // changed by nishi 2024.2.28
        set_drive_mode(1);  // set nav2 mode

    }
    anchorFinder_.release_blk();
    std::cout << ">> End Auto Map Anchor" << std::endl;
}

/*
mloop_ex(GoalList *goalList)
    goalList: ゴールリスト
*/
void MultiGoals::mloop_ex(GoalList *goalList){
    // params & variables
    _goalList = goalList;
    goalId = 0;
    t_type=0;
    mloop();
}
/*
mloop_ex2(GoalList2 *goalList2)
    goalList: ゴールリスト2
*/
void MultiGoals::mloop_ex2(GoalList2 *goalList2){
    // params & variables
    _goalList2 = goalList2;
    goalId = 0;
    t_type=1;
    mloop();
}


/*
* set_drive_mode(int func)
* func: 0 -> cmd_vel, 1 -> navi2
*/
void MultiGoals::set_drive_mode(int func){
    #if defined(USE_MOVE_BASE)
        if(func==1){
            if(mode_f_origin ==1 && mode_f==0){
                drive_=dynamic_cast<Robot_DriveCore *>(&drive_nav);
                mode_f=1;
            }
        }
        else{
            if(mode_f_origin ==1 && mode_f==1){
                drive_=dynamic_cast<Robot_DriveCore *>(&drive_cmd);
                mode_f=0;
            }
        }
    #endif
}

/*
* check_obstacle_backaround(float r_lng,int black_thresh)
*  float r_lng: ロボットの周囲の半径[M]
*  int black_thresh: black count の閾値
*  ロボットの前方の障害物をチェックして、前方が塞がっていれば、後ろ向きにする。
*/
void MultiGoals::check_obstacle_backaround(float r_lng,int black_thresh){
    get_map.get();
    drive_->get_tf(2);  
    tf2::Vector3 cur_origin = drive_->base_tf.getOrigin();  // changed by nishi 2024.2.28

    float cur_x_tmp = cur_origin.getX();    // World point(基本座標) [M]
    float cur_y_tmp = cur_origin.getY();    // World point(基本座標) [M]
    // ロボットの今の向き
    std::cout << "drive->_rz*RADIANS_F: " << drive_->_rz*RADIANS_F <<std::endl;
    // 前方チェック
    int rcx= get_map.check_obstacle(cur_x_tmp,cur_y_tmp,drive_->_rz,r_lng,0,black_thresh);
    if(rcx!=0){
        set_drive_mode(0);      // set cmd_vel mode
        // 後ろを向かせる
        drive_->rotate_off(90.0);   // changed by nishi 2024.3.7
        drive_->rotate_off(90.0);   // changed by nishi 2024.3.7

        // 再度前方チェックして、障害物がなければ、0.05[M] 前に移動

        get_map.get();
        drive_->get_tf(2);  
        cur_origin = drive_->base_tf.getOrigin();  // changed by nishi 2024.2.28

        cur_x_tmp = cur_origin.getX();    // World point(基本座標) [M]
        cur_y_tmp = cur_origin.getY();    // World point(基本座標) [M]
        // 前方チェック
        rcx= get_map.check_obstacle(cur_x_tmp,cur_y_tmp,drive_->_rz,r_lng,0,black_thresh);
        if(rcx==0){
            std::cout << "go forward 0.1[M]" <<std::endl;
            drive_->move(0.1,drive_->_rz*RADIANS_F);
        }
        set_drive_mode(1);      // set nav2 mode

    }
    else{
        //後ろをチェック
        rcx= get_map.check_obstacle(cur_x_tmp,cur_y_tmp,drive_->_rz,r_lng,2,black_thresh);
        if(rcx!=0){
            // 前に動かす
            std::cout << "go forward 0.1[M]" <<std::endl;
            set_drive_mode(0);      // set cmd_vel mode
            drive_->move(0.1,drive_->_rz*RADIANS_F);
            set_drive_mode(1);      // set nav2 mode
        }

    }
}

/*
* obstacle_escape(float r_lng,int black_thresh,float move_l)
*  float r_lng: ロボットの周囲の半径[M]
*  int black_thresh: black count の閾値
*  float move_l: 移動距離[M] =0.12
*  ロボットの四方の障害物をチェックして、障害物から少しだけ、離れる。
*/
void MultiGoals::obstacle_escape(float r_lng,int black_thresh,float move_l){
    bool all_black=true;
    get_map.get();
    drive_->get_tf(2);  
    tf2::Vector3 cur_origin = drive_->base_tf.getOrigin();  // changed by nishi 2024.2.28

    float cur_x_tmp = cur_origin.getX();    // World point(基本座標) [M]
    float cur_y_tmp = cur_origin.getY();    // World point(基本座標) [M]
    // ロボットの今の向き
    std::cout << "drive->_rz*RADIANS_F: " << drive_->_rz*RADIANS_F <<std::endl;

    int black_counts[4];
    int min_black_count=1000;
    int min_black_idx=-1;
    // 全方向をチェックします。
    for(int i=0;i<4;i++){
        black_counts[i]= get_map.check_obstacle(cur_x_tmp,cur_y_tmp,drive_->_rz,r_lng,i,0);
        if(black_counts[i] < min_black_count){
            min_black_count=black_counts[i];
            min_black_idx=i;
        }
    }

    set_drive_mode(0);      // set cmd_vel mode

    // 前方向は、障害物無し
    if(black_counts[0] <= black_thresh){
        std::cout << "stay forward" <<std::endl;
        // 後ろ方向に障害物があります。
        if(black_counts[2] > black_thresh){
            // 前に、0.1[M] 動かす。
            std::cout << "go forward 0.1[M]" <<std::endl;
            drive_->move(move_l,0.0,true);
        }
        // 左右に障害物があります。
        else if((black_counts[1] > 0 && black_counts[3] > 0) || (black_counts[1] > black_thresh) || (black_counts[3] > black_thresh)){
            // 前に、0.1[M] 動かす。
            std::cout << "go forward 0.1[M]" <<std::endl;
            drive_->move(move_l,0.0,true);
        }
    }
    // 後ろの障害物が、有りません。
    else if(black_counts[2] <= black_thresh){
        std::cout << "turn around backward" <<std::endl;
        // 後ろを向かせ、0.1[M] 動かす。
        // 後ろを向かせる
        //drive_->rotate_off(90.0);
        drive_->rotate_off(180.0);

        std::cout << "go backward 0.1[M]" <<std::endl;
        drive_->move(move_l,0.0,true);
    }
    // 左が、右より障害物が少ない
    else if(black_counts[1] < black_counts[3]){
        // 左が障害物が無い
        if(black_counts[1] <= black_thresh){
            std::cout << "turn left" <<std::endl;
            //左へ向かせる。
            drive_->rotate_off(90.0);
            // 右が、障害物あり
            if(black_counts[3] > black_thresh){
                // 前に、0.1[M] 動かす。
                std::cout << "go leftward 0.1[M]" <<std::endl;
                drive_->move(move_l,0.0,true);
            }
        }
    }
    // 右が、障害物が無い
    else if(black_counts[3] <= black_thresh){
        std::cout << "turn right" <<std::endl;
        //右へ向かせる。
        drive_->rotate_off(-90.0);
        // 左は、障害物がある。
        if(black_counts[1] > black_thresh){
            // 前に、0.1[M] 動かす。
            std::cout << "go rightward 0.1[M]" <<std::endl;
            drive_->move(move_l,0.0,true);
        }
    }
    // すべてブラックです。
    if(min_black_count > black_thresh){
        std::cout << "all closed" <<std::endl;
        // 実機だと、うごかすのは危険か?
        // 一番空いている方へ移動させる。
        switch(min_black_idx){
            case 0: // 前方が空いている
                // 前に、0.02[M] 動かす。
                std::cout << " go forward 0.02[M]" <<std::endl;
                drive_->move(0.02,0.0,true);
            break;
            case 1:
                //左へ向かせる。
                drive_->rotate_off(90.0);
                // 前に、0.02[M] 動かす。
                std::cout << " go leftward 0.02[M]" <<std::endl;
                drive_->move(0.02,0.0,true);
            break;
            case 2:
                // 後ろを向かせる
                //drive_->rotate_off(90.0);
                drive_->rotate_off(180.0);
                std::cout << " go backward 0.02[M]" <<std::endl;
                drive_->move(0.02,0.0,true);
            break;
            case 3:
                //右へ向かせる。
                drive_->rotate_off(-90.0);
                // 前に、0.02[M] 動かす。
                std::cout << " go rightward 0.02[M]" <<std::endl;
                drive_->move(0.02,0.0,true);
            break;
        }
    }
    set_drive_mode(1);      // set nav2 mode
}


/*
move(self,dist,deg)
    dist: 移動距離
    deg: 方向 [度]
*/
//void MultiGoals::move(self,dist,deg){
//    goalId = 0;
//    y = dist * math.sin(math.radians(deg)) + self.goalMsg.pose.position.y;
//    x = dist * math.cos(math.radians(deg)) + self.goalMsg.pose.position.x;
//    oz = math.radians(deg);
//    self.goalList = [[0, x,y, oz]];
//    mloop();
//}

/*
m_move(self,m_list)
    m_list
    [[func,dist,deg],...]
    func: 0 -> move dist,deg
            1 -> sleep
            2 -> get map
            50 -> set Navigation mode
            99 -> end
*/
//void MultiGoals::m_move(self,m_list){
//    int cur=0;
//    while(1):
//        if (cur >= len(m_list))
//            break;
//        if (m_list[cur][0] == 9)
//            break;
//        if (m_list[cur][0] != 0){
//            self.goalId = 0;
//            self.goalList = [[m_list[cur][0], 0, 0]];
//            mloop();
//        }
//        else{
//            dist = m_list[cur][1];
//            deg = m_list[cur][2];
//            move(dist,deg);
//        }
//        cur += 1;
//}


/*
mloop(self)
    self.goalList =[[func,x,y,d_yaw],....] or [[func,dist,d_yaw],....]
    func,x,y,d_yaw
        func: 0 -> move point x,y, and rotate d_yaw
            1 -> move point x,y only
            2 -> rotate d_yaw only   rotate_abs()
            3 -> roate_off()
            10 -> navi move x,y,d_yaw

    func,dist,d_yaw
        func: 0 -> move dist and rotate d_yaw

    func
            21 -> sleep
            22 -> get map
            23 -> map update
            30 -> auto map build
            31 -> auto map build of anchor
            50 -> set Navigation mode
            60 -> course_correct ON
            61 -> course_correct OFF
            62 -> after_correct_wait ON
            63 -> after_correct_wait OFF
            64 -> go curve ON
            65 -> go curve OFF
            66 -> set current postion as map(0,0)
            67 -> set dumper ON
            68 -> set dumper OFF
            69 -> save local cost map
            70 -> set border top-right
            71 -> set border bottom-left
            72 -> set line_w_
            73 -> set robo_r_
            80 -> navigation2 mode
            81 -> cmd_vel mode
            99 -> end
*/
void MultiGoals::mloop(){
    bool f=true;
    u_int8_t func;
    while(f){
        if(t_type==0)
            func = _goalList[goalId].func;
        else
            func = _goalList2[goalId].func;

        switch (func){
            case 0:
            case 1:
            case 2:
            case 3:
            case 10:
                mloop_sub();
                break;
            case 21:
                // sleep
                sleep(1);
                break;
            case 22:
                #if defined(USE_FUTURE_GET_MAP)
                    get_map.get(true);
                    // 障害物の判定テストです。
                    //#define TEST_KK_888
                    #if defined(TEST_KK_888)
                    {
                        drive_->get_tf(2);      // changed by nishi 2024.2.28
                        tf2::Vector3 cur_origin = drive_->base_tf.getOrigin();  // changed by nishi 2024.2.28

                        float cur_x = cur_origin.getX();    // World point(基本座標) [M]
                        float cur_y = cur_origin.getY();    // World point(基本座標) [M]
                        // ロボットの今の向き
                        std::cout << "drive->_rz*RADIANS_F: " << drive_->_rz*RADIANS_F <<std::endl;
                        // 前方チェック
                        //int rcx= get_map.check_obstacle(cur_x,cur_y,drive_->_rz,0.60,0);
                        // 右横チェック
                        //int rcx= get_map.check_obstacle(cur_x,cur_y,drive_->_rz,0.60,3);
                        // 後方チェック
                        int rcx= get_map.check_obstacle(cur_x,cur_y,drive_->_rz,0.60,2);
                    }
                    #endif

                    // GetMap::test_plot() テスト
                    //#define TEST_KK_777
                    #if defined(TEST_KK_777)
                    {
                        drive_->get_tf(2);      // changed by nishi 2024.2.28
                        tf2::Vector3 cur_origin = drive_->base_tf.getOrigin();  // changed by nishi 2024.2.28

                        float cur_x = cur_origin.getX();    // World point(基本座標) [M]
                        float cur_y = cur_origin.getY();    // World point(基本座標) [M]
                        // ロボットの今の向き
                        std::cout << "drive->_rz*RADIANS_F: " << drive_->_rz*RADIANS_F <<std::endl;
                        get_map.test_plot(cur_x,cur_y,drive_->_rz,0.60);
                    }
                    #endif

                #endif
                break;
            case 30:        // auto map build
                auto_map();
                break;
            case 31:        // auto map build of anchor
                auto_map_anchor();
                break;
            case 50:
                call_service();
                std::cout << "set Navigation Mode" << std::endl;
                break;
            case 60:
                // course correct ON
                //drive._course_correct = true;
                drive_cmd._course_correct = true;     // changed by nishi 2024.2.28
                std::cout << "course correct ON" << std::endl;
                break;
            case 61:
                // course correct OFF
                //drive._course_correct = false;
                drive_cmd._course_correct = false;     // changed by nishi 2024.2.28
                std::cout << "course correct OFF" << std::endl;
                break;
            case 62:
                // after_correct_wait ON
                //drive._after_correct_wait = true;
                drive_cmd._after_correct_wait = true; // changed by nishi 2024.2.28
                std::cout << "after_correct_wait ON" << std::endl;
                break;
            case 63:
                // after_correct_wait OFF
                //drive._after_correct_wait = false;
                drive_cmd._after_correct_wait = false;    // changed by nishi 2024.2.28
                std::cout << "after_correct_wait OFF" << std::endl;
                break;

            case 64:
                // go curve ON
                //drive._go_curve = true;
                drive_cmd._go_curve = true;   // changed by nishi 2024.2.28
                std::cout << "go curve ON" << std::endl;
                break;
            case 65:
                // go curve OFF
                //drive._go_curve = false;
                drive_cmd._go_curve = false;   // changed by nishi 2024.2.28
                std::cout << "go curve OFF" << std::endl;
                break;

            case 66:    
                // set current postion to start
                std::cout << "set current postion to start" << std::endl;

                // tf map からのロボットの距離を求める。
                //drive.get_tf(2);    // tf-map z off 2022.11.15 by nishi
                drive_->get_tf(2);    // tf-map z off 2022.11.15 by nishi

                //cur_pos = drive.base_tf.getOrigin();
                cur_pos = drive_->base_tf.getOrigin();    // changed by nishi 2024.2.28

                force_start_origin=true;
                start_pos_x = cur_pos.getX();
                start_pos_y = cur_pos.getY();

                std::cout << "start_pos_x: " << start_pos_x << std::endl;
                std::cout << "start_pos_y: " << start_pos_y << std::endl;

                break;

            case 67:    // set dumper ON
                std::cout << "set dumper ON" << std::endl;
                //drive._dumper=true;
                drive_cmd._dumper=true;     // changed by nhishi 2024.2.28
                break;

            case 68:    // set dumper OFF
                std::cout << "set dumper OFF" << std::endl;
                //drive._dumper=false;
                drive_cmd._dumper=false;     // changed by nhishi 2024.2.28
                break;
            case 69:
                std::cout << "save local cost map" << std::endl;
                //drive.navi_map_save();
                drive_->navi_map_save(); // changed by nhishi 2024.2.28
                break;

            case 70:    // set border top-right
                std::cout << "set border top-right" << std::endl;
                blobFinder_.border_def.top_r.x=_goalList[goalId].x;
                blobFinder_.border_def.top_r.y=_goalList[goalId].y;

                anchorFinder_.border_def.top_r.x=_goalList[goalId].x;
                anchorFinder_.border_def.top_r.y=_goalList[goalId].y;
                break;

            case 71:    // set border bottom-left
                std::cout << "set border bottom-left" << std::endl;
                blobFinder_.border_def.bot_l.x= _goalList[goalId].x;
                blobFinder_.border_def.bot_l.y= _goalList[goalId].y;

                anchorFinder_.border_def.bot_l.x= _goalList[goalId].x;
                anchorFinder_.border_def.bot_l.y= _goalList[goalId].y;
                break;

            case 72:    // set line_w_
                std::cout << "set line_w_" << std::endl;
                blobFinder_.line_w_=(int)_goalList[goalId].x;
                anchorFinder_.line_w_=(int)_goalList[goalId].x;
                break;

            case 73:    // set robo_r_
                std::cout << "set robo_r_" << std::endl;
                blobFinder_.robo_r_=(int)_goalList[goalId].x;
                anchorFinder_.robo_r_=(int)_goalList[goalId].x;
                break;

            case 80:    // set navigation2 mode
                std::cout << "set navigation2 mode" << std::endl;
                set_drive_mode(1);
                break;

            case 81:    // set cmd_vel mode
                std::cout << "set cmd_vel mode" << std::endl;
                set_drive_mode(0);
                break;

            case 99:
                f = false;
                break;
        }
        //time.sleep(1)
        goalId += 1;
    }
}

void MultiGoals::mloop_sub(){
    sts=0;
    int r_ct =0;

    float x,y,d_yaw,dist;

    if (t_type == 0){
        x = _goalList[goalId].x;
        y = _goalList[goalId].y;
        d_yaw = _goalList[goalId].d_yaw;
        if(force_start_origin==true){
            x += start_pos_x;
            y += start_pos_y;
        }
        if (_goalList[goalId].func == 0){
            //drive.move_abs(x,y,d_yaw);
            drive_->move_abs(x,y,d_yaw);    // changed by nishi 2024.2.28
            //self.goalMsg.pose.position.y = y;
            //self.goalMsg.pose.position.x = x;
        }
        else if (_goalList[goalId].func == 1){
            //drive.go_abs(x,y);
            drive_->go_abs(x,y);            // changed by nishi 2024.2.28
            //self.goalMsg.pose.position.y = y;
            //self.goalMsg.pose.position.x = x;
        }
        else if (_goalList[goalId].func == 2){
            //drive.rotate_abs(d_yaw);
            drive_->rotate_abs(d_yaw);      // changed by nishi 2024.2.28
        }
        else if (_goalList[goalId].func == 3){
            //drive.rotate_off(d_yaw);
            drive_->rotate_off(d_yaw);      // changed by nishi 2024.2.28
        }
        else if(_goalList[goalId].func == 10){
            //drive.navi_move(x,y,d_yaw/RADIANS_F);
            drive_->navi_move(x,y,d_yaw/RADIANS_F); // changed by nishi 2024.2.28
        }
    }
    else{
        if(_goalList2[goalId].func != 10){
            dist = _goalList2[goalId].dist;
            d_yaw = _goalList2[goalId].d_yaw;
            //drive.move(dist,d_yaw);
            drive_->move(dist,d_yaw,true);           // changed by nishi 2024.3.8
            //self.goalMsg.pose.position.y += dist * math.sin(math.radians(d_yaw));
            //self.goalMsg.pose.position.x += dist * math.cos(math.radians(d_yaw));
        }
    }
    sleep(1);   
}

#ifdef KKKKK_1
void MultiGoals::get_odom(){
    odom_msg=None;
    int cnt=30;
    float x=None;
    float y=None;
    float z=None;
    float ox=None;
    float oy=None;
    float oz=None;

    while (odom_msg is None and cnt >=0){
        try:
            if (self.use_sim_time == true)
                odom_msg = rospy.wait_for_message('/odom', Odometry, timeout=5);
            else
                odom_msg = rospy.wait_for_message('/odom_fox', Odometry, timeout=5);
        except:
            pass
        cnt-=1;
    }
    if (odom_msg != None){
        // x[M] ,y[M]
        x =self.goalMsg.pose.position.x - odom_msg.pose.pose.position.x;
        y =self.goalMsg.pose.position.y - odom_msg.pose.pose.position.y;
        z =self.goalMsg.pose.position.z - odom_msg.pose.pose.position.z;

        //ox =round(self.goalMsg.pose.orientation.x - odom_msg.pose.pose.orientation.x,4)
        //oy =round(self.goalMsg.pose.orientation.y - odom_msg.pose.pose.orientation.y,4)
        //oz =round(self.goalMsg.pose.orientation.z - odom_msg.pose.pose.orientation.z,4)
        //ow =round(self.goalMsg.pose.orientation.w - odom_msg.pose.pose.orientation.w,4)

        pe = tf.transformations.euler_from_quaternion((self.goalMsg.pose.orientation.x, self.goalMsg.pose.orientation.y, self.goalMsg.pose.orientation.z, self.goalMsg.pose.orientation.w));
        //print 'pe[0],pe[1],pe[2]=',pe[0],pe[1],pe[2]
        oe = tf.transformations.euler_from_quaternion((odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w));
        ox = pe[0]-oe[0];
        oy = pe[1]-oe[1];
        oz = pe[2]-oe[2];

        print 'differ x,y,z:ox,oy,oz =',round(x,4),round(y,4),round(z,4),':',round(ox,4),round(oy,4),round(oz,4)
        print 'ok'
    }
    else
        print 'error';
    return x,y,z,ox,oy,oz;
}
#endif

/*
* https://qiita.com/hoshianaaa/items/74b0ffbcbf97f4938a4d
* http://forestofazumino.web.fc2.com/ros/ros_service.html
*/
void MultiGoals::call_service(){
    #ifdef FUTURE_USE_3
    ros::ServiceClient client = nh_.serviceClient<std_srvs::Empty>("/rtabmap/set_mode_localization");

    std_srvs::Empty::Request req;             // リクエストの生成
    std_srvs::Empty::Response resp;           // レスポンスの生成

    bool result = client.call(req, resp);
    if(result){
        //ROS_INFO_STREAM("Recive response!");
        std::cout << "service OK" << std::endl;
    }
    else{
        //ROS_INFO_STREAM("Error!");
        std::cout << "service Error!!" << std::endl;
    }
    #endif
}

