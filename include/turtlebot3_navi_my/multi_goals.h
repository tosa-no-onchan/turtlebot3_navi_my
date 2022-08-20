/*
multi_goals.h

*/

#ifndef MULTI_GOALS_H
#define MULTI_GOALS_H

#include <math.h>
#include <cstdio>
#include <vector>
#include <list>
#include <algorithm>
#include <functional>

//#include "ros/console.h"
//#include "nav_msgs/GetMap.h"
#include <tf/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Quaternion.h>


#include <iostream>

#include <ros/ros.h>
#include <ros/topic.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include "std_srvs/Empty.h"
#include <unistd.h>

//#include <math.h>

#if defined(USE_MOVE_BASE)
    #include "turtlebot3_navi_my/robot_driveMB.h"
#else
    #include "turtlebot3_navi_my/robot_drive.h"
#endif

#include <nav_msgs/OccupancyGrid.h>

#include "com_def.h"
#include "com_lib.h"
#include "Labeling.h"


#include <opencv2/opencv.hpp>
//#define USE_MAP_SAVER 
// map_server/map_saver の画像を使う時は、こちらを使って下さい。
#ifdef USE_MAP_SAVER
    #define FREE_AREA 254
    #define UNKNOWN_AREA 255

// オンちゃん独自の  保管画像を使う時は、こちら
#else
    #define FREE_AREA 0xff
    #define UNKNOWN_AREA 0x80
#endif

#define COLOR_1 40

#define G_POINTS_MAX 20


int compare_int(const void *a, const void *b);
bool compare_Gpoint_dist_min(Gpoint &s1,Gpoint &s2);
bool compare_Gpoint_dist_max(Gpoint &s1,Gpoint &s2);
void condense_Gpoint(std::vector<Gpoint> *gp);
bool find_Gpoint(float x,float y,std::vector<Gpoint> &gp);
//void gridToWorld(int gx, int gy, float& wx, float& wy,Yaml& yaml);
void gridToWorld(int gx, int gy, float& wx, float& wy,MapM& mapm);
//bool worldToGrid(float wx, float wy,int& gx,int& gy,Yaml& yaml);
bool worldToGrid(float wx, float wy,int& gx,int& gy,MapM& mapm);

/*----------------------------
- class AnchorFinder 
----------------------------*/
class AnchorFinder{
private:
    bool view_f=false;
    bool view_f_m=false;
    int  line_w_ = 5;  // ラインの幅 -> grid size [dot]  0.05[m] * 5 = 25[cm]

    // blob 重心
    double x_g;
    double y_g;

    cv::Mat img_lab_;

    // Map
    //Yaml yaml_;
    MapM mapm_;

    // 25[cm] Grid
    //Yaml grid_yaml_;
    MapM grid_mapm_;
    // grid ブロック table サイズ
    int size_x_;
    int size_y_;

    // 50[cm] Block
    cv::Mat blk_;
    //Yaml blk_yaml_;
    MapM blk_mapm_;

    bool blk_f_=false;
    //int blk_size_;   // blk_ 1 block size [cm] / 1dot = 50[cm]
    //int blk_w_;      // blk_  width
    //int blk_h_;      // blk_ height

public:
    // ロボットの第一候補位置(基本座標)
    double abs_x_;
    double abs_y_;
    double yaw_;

    int point_n_;   // ラベル総数

    u_char block_mode = 0;  // 走査ブロクモード 0: 全て対象 1: ブロック1 が対象 2: ブロック2 が対象
    float block_line_x = 0.0; // ブロック分け X軸 ライン

    std::vector<Gpoint> g_points_;
    std::vector<Gpoint> g_points1;
    std::vector<Gpoint> g_points2;

    std::vector<Gpoint> g_points_black;

    int anchor_;

    //std::list<Gpoint> g_points_list;
    //std::list<Gpoint> g_points1_list;
    //std::list<Gpoint> g_points2_list;

    // ロボットの行動範囲 / ボーダー定義
    // top-left (+x,+y) / bottom-right(-x,-y) [M]
    BorderBox border_def={{10.525,10.525},        //  top-left(+x,+y)
                         {-10.525,-10.525}};      //  bottom-right(-x,-y)

    AnchorFinder(){}
    ~AnchorFinder(){
        if(blk_f_ == true){
            //delete[] blk_;
            blk_f_=false;
        }
    }
    void init();
    void release_blk(){
        if(blk_f_ == true){
            //delete[] blk_;
            blk_f_=false;
        }
    }

    void sort_blob(float cur_x,float cur_y);
    void check(cv::Mat mat_map,MapM &mapm,float cur_x,float cur_y);
    void anchoring(cv::Mat &mat_blob2,float cur_x,float cur_y);
    bool anchor_put(int gx,int gy,float cur_x,float cur_y);
    bool check_Border(float x,float y);
    void mark_blk_world(float wx,float wy);
    void mark_blk(int bx,int by);
    //void gridToWorld(int gx, int gy, float& wx, float& wy);
    //void worldToBlock(float wx, float wy,int& bx,int& by);
    void save_blk();
};

/*----------------------------
- class BlobFinder
----------------------------*/
class BlobFinder{
private:
    bool view_f=false;
    bool view_f_m=false;
    int   line_w_ = 5;  // ラインの幅 -> grid size [dot]  0.05[m] * 5 = 25[cm]

    // blob 重心
    double x_g;
    double y_g;

    cv::Mat img_lab_;

public:
    // ロボットの第一候補位置(基本座標)
    double abs_x_;
    double abs_y_;
    double yaw_;

    int point_n_;   // ラベル総数

    u_char block_mode = 0;  // 走査ブロクモード 0: 全て対象 1: ブロック1 が対象 2: ブロック2 が対象
    float block_line_x = 0.0; // ブロック分け X軸 ライン

    std::vector<Gpoint> g_points_;
    std::vector<Gpoint> g_points1;
    std::vector<Gpoint> g_points2;

    std::vector<Gpoint> g_points_black;

    //std::list<Gpoint> g_points_list;
    //std::list<Gpoint> g_points1_list;
    //std::list<Gpoint> g_points2_list;

    // ロボットの行動範囲 / ボーダー定義
    // top-left (+x,+y) / bottom-right(-x,-y) [M]
    BorderBox border_def={{10.0,10.0},        //  top-left(+x,+y)
                         {-10.0,-10.0}};      //  bottom-right(-x,-y)

    BlobFinder(){}

    void sort_blob(float cur_x,float cur_y);
    //void check(cv::Mat mat_map,Yaml &yaml,float cur_x,float cur_y);
    void check(cv::Mat mat_map,MapM &mapm,float cur_x,float cur_y);
    bool check_Border(float x,float y);
};

/*----------------------------
- class Grid
----------------------------*/
class Grid{
private:
    u_int32_t width_;
    u_int32_t height_;
    float resolution_;
    double origin_x_, origin_y_;    // map 左下 (origin_x_, origin_y_)
    //double x0_,y0_;                 // 左上 原点 (x0_,y0_)

    int line_w_;
    int size_x_;
    int size_y_;
    //boost::shared_ptr<int8_t[]> blk_;
    //std::shared_ptr<int8_t[]> blk_;
    int8_t* blk_=nullptr;

public:
    bool init_ok=false;
    Grid(){}

    void init(nav_msgs::MapMetaData map_info,int line_w,std::vector<int8_t> data);
    void updateGrid(std::vector<int8_t> data);
    void saveGrid();

    void gridToWorld(unsigned int gx, unsigned int gy, double& wx, double& wy);
    bool worldToGrid(double wx, double wy, unsigned int& gx, unsigned int& gy);

    ~Grid(){
        if (blk_ != nullptr){
            delete[] blk_;
            blk_ = nullptr;
        }
    }
};

/*----------------------------
- class GetMap
----------------------------*/
class GetMap
{
private:
    ros::NodeHandle nh_;

    ros::Subscriber _sub;

    int _free_thresh;

    int _line_w;
    int _car_r;
    bool _match_rviz;

    std::string _map_frame;

    nav_msgs::MapMetaData map_info;
    float resolution;
    int free_thresh;
    double org_x,org_y;
    //int x_size,y_size; 

    //Grid grid_;

    bool init_ok=false;

public:

    cv::Mat mat_map_;
    cv::Mat mat_bin_map_;   // map 障害物の　2値化
    cv::Mat mat_bin_free_map_;  // map 非障害物の 2値化
    //Yaml yaml_;
    MapM mapm_;

    GetMap(){}

    void init(ros::NodeHandle &nh,std::string map_frame="map");
 
    /*
    * https://answers.ros.org/question/293890/how-to-use-waitformessage-properly/
    * http://docs.ros.org/en/lunar/api/nav_msgs/html/msg/OccupancyGrid.html
    */
    void get();


    /*
    * conv_fmt2(nav_msgs::OccupancyGrid_ map_msg)
    */
    void conv_fmt2(boost::shared_ptr<const nav_msgs::OccupancyGrid_<std::allocator<void>>> map);

    void saveMap(boost::shared_ptr<const nav_msgs::OccupancyGrid_<std::allocator<void>>> map);

    void check_collision(float x,float y,float &ox,float &oy,int func=0);

    #ifdef XXXX_X
    void conv_dot2grid(self,x,y);
    /*
    convert Grid to Meter World
    */
    void conv_grid2meter(self,gx,gy,f_or_l);
    /*
    Rviz と同じ図形 にした場合。
    左上: M(+x,+y)    右上: M(-x,+y)
    左下: M(+x,-y)    右下: M(-x,-y)
    */
    void conv_grid2meter_m_p(self,gx,gy,f_or_l);
    /*
    Map データの図形 : y =  x の線対称の図形(x,y の入れ替え)
    左上: M(-x,-y)    右上: M(-x,+y)
    左下: M(+x,-y)    右下: M(+x,+y)
    */
    void conv_grid2meter_m_m(self,gx,gy,f_or_l);

    void get_rotation_matrix(self,rad);
    void np_resize(self,m_grid,req_size);
    #endif
};


/*----------------------------
- class MultiGoals
----------------------------*/
class MultiGoals
{
private:
    int sts;
    int goalId;

    u_int8_t t_type;

    //! The node handle we'll be using
    ros::NodeHandle nh_;

    GoalList *_goalList;
    GoalList2 *_goalList2;

    bool force_start_origin;
    float start_pos_x;
    float start_pos_y;
    float start_pos_z;

    tf::Vector3 cur_pos;

    BlobFinder blobFinder_;
    AnchorFinder anchorFinder_;

public:

    GetMap get_map;

    //RobotNavi navi;

    #if defined(USE_MOVE_BASE)
        RobotDriveMB drive;
    #else
        RobotDrive drive;
    #endif

    MultiGoals(){}

    //void init( map_frame,get_map,use_sim_time){
    void init(ros::NodeHandle &nh);

    void auto_map();

    void auto_map_anchor();

    /*
    mloop_ex(GoalList *goalList)
        goalList: ゴールリスト
    */
    void mloop_ex(GoalList *goalList);
    /*
    mloop_ex2(GoalList2 *goalList2)
        goalList: ゴールリスト2
    */
    void mloop_ex2(GoalList2 *goalList2);

    /*
    move(self,dist,deg)
        dist: 移動距離
        deg: 方向 [度]
    */
    //void move(self,dist,deg){
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
    //void m_move(self,m_list){
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
              99 -> end
    */
    void mloop();

    void mloop_sub();

    #ifdef KKKKK_1
    void get_odom(){
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
    void call_service();

};


#endif      // MULTI_GOALS_H
