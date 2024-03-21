/*
* turtlebot3_navi_my/src/pro_control_sub.cpp
*
*/

#include "turtlebot3_navi_my/pro_control_sub.hpp"

//using namespace std::chrono_literals;

using std::placeholders::_1;


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
* class GetMap
--------------------------*/
//void GetMap::init(ros::NodeHandle &nh,std::string map_frame)
void GetMap::init(std::shared_ptr<rclcpp::Node> node,int func,std::string map_frame)
{
    //nh_ = nh;
    node_=node;
    map_frame_ = map_frame;
    func_=func;

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

    // map は、不定期に、publish されている場合は、 call back で、取得する。
    if(func_==0){
        subscript_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_frame, 10, std::bind(&GetMap::topic_callback, this, _1));
    }

}

void GetMap::topic_callback(const nav_msgs::msg::OccupancyGrid & map_msg)
{
    //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    #if defined(USE_MAP_INT_TRACE)
        std::cout << "GetMap::map_msg.header.frame_id=" << map_msg.header.frame_id << std::endl;
    #endif

    //std::cout << "map_msg.header" << map_msg.header << std::endl; 
    //std::cout << "map_msg.info" << map_msg.info << std::endl;

    //auto info = map_msg.info;
    //auto data = map_msg.data;
    //printf("%s",info);
    //printf("%s",data);
    if(map_ptr_cnt_==0){
        map_ptr_=std::make_shared<nav_msgs::msg::OccupancyGrid>(map_msg);
        map_ptr_cnt_=1;
    }
    else{
        map_ptr_.reset();
        map_ptr_=std::make_shared<nav_msgs::msg::OccupancyGrid>(map_msg);
    }
}


/*
* https://answers.ros.org/question/293890/how-to-use-waitformessage-properly/
* http://docs.ros.org/en/lunar/api/nav_msgs/html/msg/OccupancyGrid.html
* http://docs.ros.org/en/jade/api/map_server/html/map__saver_8cpp_source.html
* https://boostjp.github.io/tips/smart_ptr.html
* https://yomi322.hateblo.jp/entry/2012/04/17/223100
* https://qiita.com/usagi/items/3563ddb01e4eb342485e
*/
void GetMap::get(bool save_f){
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
        nav_msgs::msg::OccupancyGrid *map_dt=map_ptr_.get();
        map = *map_dt;
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

        //x_size = map->info.width / _line_w;
        //y_size = map->info.height / _line_w;

        std::cout << " free_thresh=" << free_thresh << std::endl;

        //grid_.init(map->info,_line_w,map->data);

        //conv_fmt2(map);
        saveMap(map,save_f);

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
}

/*
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
            unsigned int i = x + (map.info.height - y - 1) * map.info.width;
            if (map.data[i] == 0) { //occ [0,0.1)
                //fputc(254, out);
                //fputc(255, out);    // 0xff  white
                if(save_f) fputc(FREE_AREA, out);
                mat_map_.data[i] = FREE_AREA;
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
                mat_map_.data[i] = UNKNOWN_AREA;
            } 
            else{                       // 障害領域
                if(save_f) fputc(100-map.data[i],out);
                mat_map_.data[i] = 100-map.data[i];
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

    //ロボットの現在位置を、マスクの中心にします。
    center_p.x = px;
    center_p.y = py;

    // Mask画像 を作成
    mask = cv::Mat::zeros(mapm_.height, mapm_.width, CV_8UC1);
    // 円弧、扇形を描く
	// ellipse(画像, 中心座標, Size(x径, y径), 楕円の回転角度, 始点角度, 終点角度, 色, 線幅, 連結)

    //int rr=12;
    int rr=(int)(r_lng/0.05);

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
        cv::imshow("img", mask);
        cv::waitKey(0);
        cv::destroyAllWindows();
    #endif

    //#define TEST_KK2_A
    #if defined(TEST_KK2_A)
        cv::imshow("画像", mat_bin_map_);
        cv::waitKey(0);
        cv::destroyAllWindows();
    #endif

    // 障害物 2値化画像に 円のマスクを実施
    mat_bin_map_.copyTo(result2,mask);

    //#define TEST_KK2_A2
    #if defined(TEST_KK2_A2)
        cv::imshow("画像", result2);
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


/*
test_plot()
    float x,y: ロボット位置(基準座標) [M]
    float r_yaw: 基準座標での角度。 [rad]
    float robot_r=0.3 : ロボット半径
    ロボットの位置をマップにプロットする。
*/
bool GetMap::test_plot(float x,float y,float r_yaw,float robot_r){
    get();

    std::cout << "GetMap::test_plot()" << std::endl;

    std::cout << "mapm_.origin[0]:"<< mapm_.origin[0]<< " mapm_.origin[1]:"<< mapm_.origin[1] << "mapm_.resolution:"<< mapm_.resolution  << std::endl;

    cv::Point center_p; // 円の中心位置
    cv::Mat my_map;

    // real world 座標を、Mat map 座標に変換 
    int px = (int)((x - mapm_.origin[0]) / mapm_.resolution);
    int py = (int)((y - mapm_.origin[1]) / mapm_.resolution);

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

    cv::namedWindow("GetMap::test_plot", cv::WINDOW_NORMAL);

    cv::imshow("GetMap::test_plot", my_map);
    int rc = cv::waitKey(1000);
    cv::destroyWindow("GetMap::test_plot");

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
    getmap_->check_collision(wx0,wy0,wx,wy,robo_r_,1);  // changed by nishi 2024.3.1

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
