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

}

void GetMap::topic_callback(const nav_msgs::msg::OccupancyGrid & map_msg)
{
    //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    #define USE_MAP_INT_TRACE
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
bool GetMap::get(bool save_f){
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
    return is_successful;
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
void GetMap::test_plot(float x,float y,float r_yaw,float robot_r){

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
    int key = cv::waitKey(1000);
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


