#include "turtlebot3_navi_my/multi_goals.h"

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
* class BlobFinder
* find_Gpoint()
--------------------------*/
bool BlobFinder::check_Border(float x,float y){
    if(x > border_def.top_l.x || x < border_def.bot_r.x)
        return false;
    if(y > border_def.top_l.y || y < border_def.bot_r.y)
        return false;
    return true;
}

/*-------------------------
* class BlobFinder
* find_Gpoint()
--------------------------*/
bool BlobFinder::find_Gpoint(float x,float y,std::vector<Gpoint> &gp){
    for (int i=0;i<gp.size();i++){
        if(gp.at(i).x == x && gp.at(i).y == y){
            return true;
        }
    }
    return false;
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
void BlobFinder::check(cv::Mat mat_map,Yaml &yaml,float cur_x,float cur_y){
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

    // 90度回転しているので補正は、どうする?
    // blob の重心は、左上が、原点です
    #ifdef USE_90_ADJUST
        abs_y_ = (x_g + 0.5) * (double)line_w_ * yaml.resolution + yaml.origin[0];
        abs_x_ = (y_g + 0.5) * (double)line_w_ * yaml.resolution + yaml.origin[1];
    #else
        abs_x_ = (x_g + 0.5) * (double)line_w_ * yaml.resolution + yaml.origin[0];
        abs_y_ = (y_g + 0.5) * (double)line_w_ * yaml.resolution + yaml.origin[1];
    #endif
    yaw_ = yaml.origin[3];

	std::cout <<"yaml.resolution="<< yaml.resolution << std::endl;

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
        // 90度回転しているので補正は、どうする?
        #ifdef USE_90_ADJUST
            y0 = (x_g + 0.5) * (double)line_w_ * yaml.resolution + yaml.origin[0];
            x0 = (y_g + 0.5) * (double)line_w_ * yaml.resolution + yaml.origin[1];
        #else
            x0 = (x_g + 0.5) * (double)line_w_ * yaml.resolution + yaml.origin[0];
            y0 = (y_g + 0.5) * (double)line_w_ * yaml.resolution + yaml.origin[1];
        #endif

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

/*-------------------------
* class Grid
--------------------------*/
void Grid::init(nav_msgs::MapMetaData map_info,int line_w,std::vector<int8_t> data){

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
        ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
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


/*-------------------------
* class GetMap
--------------------------*/
void GetMap::init(ros::NodeHandle &nh,std::string map_frame)
{
    nh_ = nh;
    _map_frame = map_frame;

    //_sub = nh.subscribe(_map_frame, 1);
    //self.map_info = None
    //self.map_data = None
    //self.grid = None
    //self.resolution = None
    _line_w = 5;  // ラインの幅 -> grid size [dot]  0.05[m] * 5 = 25[cm]
    _car_r = 4;    // ロボットの回転径 [dot]
    _match_rviz = true;      // True / Flase  -> Rviz の画像と同じにする / しない
}

/*
* https://answers.ros.org/question/293890/how-to-use-waitformessage-properly/
* http://docs.ros.org/en/lunar/api/nav_msgs/html/msg/OccupancyGrid.html
* http://docs.ros.org/en/jade/api/map_server/html/map__saver_8cpp_source.html
* https://boostjp.github.io/tips/smart_ptr.html
* https://yomi322.hateblo.jp/entry/2012/04/17/223100
* https://qiita.com/usagi/items/3563ddb01e4eb342485e
*/
void GetMap::get(){
    int cnt=3;
    boost::shared_ptr<const nav_msgs::OccupancyGrid_<std::allocator<void>>> map=nullptr;

    while (map==nullptr && cnt >0){
        // auto map_msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map",nh_,ros::Duration(1.0));
        //printf("%s",map_msg);  // コンパイルエラーで、型が判る
        //boost::shared_ptr<const nav_msgs::OccupancyGrid_<std::allocator<void>>>

        map = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(_map_frame,nh_,ros::Duration(1.0));
        //std::cout << "map_msg->header" << map_msg->header << std::endl; 
        //std::cout << "map_msg->info" << map_msg->info << std::endl;
        cnt--;
    }
    
    if (map != nullptr){
        map_info = map->info;
        std::cout << "map->info=" << map->info << std::endl;
        free_thresh = int(0.196 * 255);



        std::cout << "map->info.width=" << map->info.width << std::endl;         // 225
        std::cout << "map->info.height=" << map->info.height << std::endl;       // 141

        resolution = map->info.resolution;
        std::cout << "map->info.resolution=" << resolution << std::endl;        // 0.05

        // ロボット位置
        org_x = map->info.origin.position.x;
        org_y = map->info.origin.position.y;
        std::cout << "org_x=" << org_x << std::endl;
        std::cout << "org_y=" << org_y << std::endl;

        //x_size = map->info.width / _line_w;
        //y_size = map->info.height / _line_w;

        std::cout << "free_thresh=" << free_thresh << std::endl;

        //grid_.init(map->info,_line_w,map->data);

        //conv_fmt2(map);
        saveMap(map);


        //2. 障害物を、2値画像 and 反転します。 add by nishi 2022.8.13
        //int thresh = 120;
        //int thresh = 2;
        int thresh = 10;        // 障害物の値 0-100 / 未知領域:128  / 自由領域:255
        //threshold(gry, img_dst, thresh, 255, cv::THRESH_BINARY);
        // 障害物 < 10 だけを、白にします。
        cv::threshold(mat_map_, mat_bin_map_, thresh, 255, cv::THRESH_BINARY_INV);


        std::cout << "GetMap ok" << std::endl;

        // BlobFinder call
        //std::cout << "call BlobFinder" << std::endl;
        //blobFinder_.check(mat_map_);
 
    }
    else{
        std::cout << "error" << std::endl;
    }
}

/*
*
* http://docs.ros.org/en/jade/api/map_server/html/map__saver_8cpp_source.html
*
* http://docs.ros.org/en/lunar/api/nav_msgs/html/msg/OccupancyGrid.html
*
*/
void GetMap::saveMap(boost::shared_ptr<const nav_msgs::OccupancyGrid_<std::allocator<void>>> map){
    std::string mapname_ ="/home/nishi/map_builder";
    std::string mapdatafile = mapname_ + ".pgm";
    //ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
    FILE* out = fopen(mapdatafile.c_str(), "w");
    if (!out)
    {
        ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
        return;
    }

    mat_map_ = cv::Mat::zeros(map->info.height,map->info.width,CV_8U);

    fprintf(out, "P5\n# CREATOR: Map_generator.cpp %.3f m/pix\n%d %d\n255\n",
            map->info.resolution, map->info.width, map->info.height);
    for(unsigned int y = 0; y < map->info.height; y++) {
        for(unsigned int x = 0; x < map->info.width; x++) {
            unsigned int i = x + (map->info.height - y - 1) * map->info.width;
            if (map->data[i] == 0) { //occ [0,0.1)
                //fputc(254, out);
                //fputc(255, out);    // 0xff  white
                fputc(FREE_AREA, out);
                mat_map_.data[i] = FREE_AREA;
            } 
            //else if (map->data[i] == +100) { //occ (0.65,1]
            //    fputc(000, out);
            //} 
            //else { //occ [0.1,0.65]
            //    fputc(205, out);
            //}
            else if (map->data[i] < 0) {     // 未チェック領域
                //fputc(128, out);
                fputc(UNKNOWN_AREA, out);
                mat_map_.data[i] = UNKNOWN_AREA;
            } 
            else{                       // 障害領域
                fputc(100-map->data[i],out);
                mat_map_.data[i] = 100-map->data[i];
            }
        }
    }

    fclose(out);


    std::string mapmetadatafile = mapname_ + ".yaml";
    ROS_INFO("Writing map occupancy data to %s", mapmetadatafile.c_str());
    FILE* yaml_fp = fopen(mapmetadatafile.c_str(), "w");
 
 
    /*
resolution: 0.100000
origin: [0.000000, 0.000000, 0.000000]
#
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196

    */

    geometry_msgs::Quaternion orientation = map->info.origin.orientation;
    tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
    double yaw, pitch, roll;
    mat.getEulerYPR(yaw, pitch, roll);

    yaml_.resolution=map->info.resolution;
    yaml_.origin[0] = map->info.origin.position.x;
    yaml_.origin[1] = map->info.origin.position.y;
    yaml_.origin[2] = yaw;


    fprintf(yaml_fp, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
            mapdatafile.c_str(), map->info.resolution, map->info.origin.position.x, map->info.origin.position.y, yaw);

    fclose(yaml_fp);

    //ROS_INFO("Done\n");
    //saved_map_ = true;

}

/*-------------------------
* class GetMap
* check_collision()
--------------------------*/
void GetMap::check_collision(float x,float y,float &ox,float &oy){
    ox=x;
    oy=y;

    cv::Point center_p; // 円の中心位置
    int r =5;      // 円の半径  30 -> 1.5[M]      7 ->  0.35[M]  5->0.25[M]

    cv::Mat result,result2,mask;

    //x0 = (x_g + 0.5) * (double)line_w_ * yaml.resolution + yaml.origin[0];
    //y0 = (y_g + 0.5) * (double)line_w_ * yaml.resolution + yaml.origin[1];

    // 基本座標を、Mat map 座標に変換 
    int px = (int)((x - yaml_.origin[0]) / yaml_.resolution);
    int py = (int)((y - yaml_.origin[1]) / yaml_.resolution);

    //ロボットの移動先を、マスクの中心にします。
    center_p.x = px;
    center_p.y = py;

    // Mask画像 を作成
    mask = cv::Mat::zeros(mat_bin_map_.rows, mat_bin_map_.cols, CV_8UC1);

    // ロボットの移動先を中心にした、半径r の円を描きます。
    cv::circle(mask, center_p, r, cv::Scalar(255),-1);

    // 障害物 2値化画像に 円のマスクを実施
    mat_bin_map_.copyTo(result2,mask);

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
        float x_gb = ((float)x_g + 0.5) * yaml_.resolution + yaml_.origin[0];
        float y_gb = ((float)y_g + 0.5) * yaml_.resolution + yaml_.origin[1];
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
            return;
        }

        float theta_r = std::atan2(off_target_y,off_target_x);   //  [ragian]

        float dx,dy;
        if(white_cnt < 10){
            // 5.2  (x,y) から、(x_gb,y_gb) とは逆方向に 5[cm] ずらす。
            dx = std::cos(theta_r) * 0.05;
            dy = std::sin(theta_r) * 0.05;
        }
        else{
            // 5.2  (x,y) から、(x_gb,y_gb) とは逆方向に 10[cm] ずらす。
            dx = std::cos(theta_r) * 0.1;
            dy = std::sin(theta_r) * 0.1;
        }

        ox=x+dx;
        oy=y+dy;

        std::cout << "ox=" << ox <<" oy="<< oy << std::endl;

    }

}


/*
* conv_fmt2(nav_msgs::OccupancyGrid_ map_msg)
*/
void GetMap::conv_fmt2(boost::shared_ptr<const nav_msgs::OccupancyGrid_<std::allocator<void>>> map){
    
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


#ifdef XXXX_X
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


/*
* class MultiGoals
*/
//void init( map_frame,get_map,use_sim_time){
void MultiGoals::init(ros::NodeHandle &nh){
    nh_=nh;
    //navi.init(nh,2);
    drive.init(nh,true);

    get_map.init(nh);

    goalId = 0;
    sts=0;
    t_type=0;

    force_start_origin=false;

    start_pos_x = start_pos_y = start_pos_z = 0.0;

    sleep(1);
}

/*
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
        get_map.get();


        std::cout << "auto_map() #6 call drive.get_tf(2)" << std::endl;
        drive.get_tf(2);

        tf::Vector3 cur_origin = drive.base_tf.getOrigin();
        float cur_x = cur_origin.getX();
        float cur_y = cur_origin.getY();

        std::cout << "auto_map() #7 call blobFinder_.check()" << std::endl;
        // Find Next Unkonown Blob
        blobFinder_.check(get_map.mat_map_,get_map.yaml_,cur_x,cur_y);

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
                // 此処で、走査先の障害物との距離をチェック
                get_map.check_collision(g_ponts_ptr->at(j).x,g_ponts_ptr->at(j).y,ox,oy);

                //drive.comp_dad(g_ponts_ptr->at(j).x,g_ponts_ptr->at(j).y,dist, r_yaw, r_yaw_off);
                drive.comp_dad(ox,oy,dist, r_yaw, r_yaw_off);

                // Navi move
                //drive.navi_move(blobFinder_.abs_x_+off,blobFinder_.abs_y_+off,r_yaw);
                //if(drive.navi_move(g_ponts_ptr->at(j).x+off,g_ponts_ptr->at(j).y+off,r_yaw,r_yaw_off)==false){
                if(drive.navi_move(ox+off,oy+off,r_yaw,r_yaw_off)==false){
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
            cur_origin = drive.base_tf.getOrigin();
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
        drive.rotate_off(30.0);
        drive.rotate_off(-60.0);
        drive.rotate_off(30.0);

    }
    std::cout << ">> End Auto Map" << std::endl;

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
            2 -> rotate d_yaw only
            10 -> navi move x,y,d_yaw

    func,dist,d_yaw
        func: 0 -> move dist and rotate d_yaw

    func
            21 -> sleep
            22 -> get map
            23 -> map update
            30 -> auto map build
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
            70 -> set border top-left
            71 -> set border bottom-right
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
            case 10:
                mloop_sub();
                break;
            case 21:
                // sleep
                sleep(1);
                break;
            case 22:
                get_map.get();
                break;
            case 30:        // auto map build
                auto_map();
                break;
            case 50:
                call_service();
                std::cout << "set Navigation Mode" << std::endl;
                break;
            case 60:
                // course correct ON
                drive._course_correct = true;
                std::cout << "course correct ON" << std::endl;
                break;
            case 61:
                // course correct OFF
                drive._course_correct = false;
                std::cout << "course correct OFF" << std::endl;

            case 62:
                // after_correct_wait ON
                drive._after_correct_wait = true;
                std::cout << "after_correct_wait ON" << std::endl;
                break;
            case 63:
                // after_correct_wait OFF
                drive._after_correct_wait = false;
                std::cout << "after_correct_wait OFF" << std::endl;
                break;

            case 64:
                // go curve ON
                drive._go_curve = true;
                std::cout << "go curve ON" << std::endl;
                break;
            case 65:
                // go curve OFF
                drive._go_curve = false;
                std::cout << "go curve OFF" << std::endl;
                break;

            case 66:    // set current postion to start

                std::cout << "set current postion to start" << std::endl;

                drive.get_tf();

                cur_pos = drive.base_tf.getOrigin();

                force_start_origin=true;
                start_pos_x = cur_pos.getX();
                start_pos_y = cur_pos.getY();

                break;

            case 67:    // set dumper ON
                std::cout << "set dumper ON" << std::endl;
                drive._dumper=true;
                break;

            case 68:    // set dumper OFF
                std::cout << "set dumper OFF" << std::endl;
                drive._dumper=false;
                break;
            case 69:
                std::cout << "save local cost map" << std::endl;
                drive.navi_map_save();
                break;

            case 70:    // set border top-left
                std::cout << "set border top-left" << std::endl;
                blobFinder_.border_def.top_l.x=_goalList[goalId].x;
                blobFinder_.border_def.top_l.y=_goalList[goalId].y;
                break;

            case 71:    // set border bottom-right
                std::cout << "set border bottom-right" << std::endl;
                blobFinder_.border_def.bot_r.x= _goalList[goalId].x;
                blobFinder_.border_def.bot_r.y= _goalList[goalId].y;
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
            drive.move_abs(x,y,d_yaw);
            //self.goalMsg.pose.position.y = y;
            //self.goalMsg.pose.position.x = x;
        }
        else if (_goalList[goalId].func == 1){
            drive.go_abs(x,y);
            //self.goalMsg.pose.position.y = y;
            //self.goalMsg.pose.position.x = x;
        }
        else if (_goalList[goalId].func == 2){
            drive.rotate_abs(d_yaw);
        }
        else if(_goalList[goalId].func == 10){
            drive.navi_move(x,y,d_yaw/RADIANS_F);
        }
    }
    else{
        if(_goalList2[goalId].func != 10){
            dist = _goalList2[goalId].dist;
            d_yaw = _goalList2[goalId].d_yaw;
            drive.move(dist,d_yaw);
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
}

