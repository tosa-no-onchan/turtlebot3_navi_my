/*
multi_goals4_cmd_vel.cpp
https://wiki.ros.org/pr2_controllers/Tutorials/Using%20the%20base%20controller%20with%20odometry%20and%20transform%20information

https://www.k-cube.co.jp/wakaba/server/func/math_h.html

https://answers.ros.org/question/50113/transform-quaternion/

build
$ catkin_make --pkg turtlebot3_navi_my

$ rosrun turtlebot3_navi_my multi_goals4_cmd_vel
*/

#include <iostream>

#include <ros/ros.h>
#include <ros/topic.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include "std_srvs/Empty.h"
#include <unistd.h>

//#include <math.h>

#include "turtlebot3_navi_my/robot_drive.h"

#include <nav_msgs/OccupancyGrid.h>
//from nav_msgs.msg import OccupancyGrid,Odometry

// https://progsennin.com/c-initstructarray/221/


std::string map_frame="map";
bool use_sim_time = false;

/*------------------------------------------------
 GoalList
  func,x,y,d_yaw
  func: 0 -> move point x,y and rotate d_yaw
       1 -> move point x,y only
       2 -> rotate d_yaw only
       21 -> sleep
       22 -> get map
       50 -> set Navigation mode
       99 -> end
--------------------------------------------------*/
typedef struct {
    u_int8_t func;
    float x;
    float y;
    float d_yaw;
} GoalList;

/*------------------------------------------------
 GoalList2
  func,x,dist,d_yaw
  func: 0 -> move dist and rotate d_yaw
       1 -> move point x,y only
       2 -> rotate d_yaw only
       21 -> sleep
       22 -> get map
       50 -> set Navigation mode
       99 -> end
--------------------------------------------------*/
typedef struct {
    u_int8_t func;
    float dist;
    float d_yaw;
} GoalList2;


GoalList goallist[] ={
            {60, 0.0, 0.0, 0.0},      // course correct ON
            {0, 0.0, 0.0, 0.0},   // go (0.0,0.0) and rotate 0
            {2, 0.0, 0.0, 90.0},  // rotate 90
            {2, 0.0, 0.0, 180.0},  // rotate 180
            {2, 0.0, 0.0, 270.0},  // rotate 270
            {2, 0.0, 0.0, 360},   // rotate 360

            //{50,0.0,0.0, 0.0},      // set Navigation mode
            //{22,0.0,0.0, 0.0},      // get map
            //{99,0.0,0.0, 0.0},      // end

            {0,1.0,0.0, 0.0},      // go (1.0,0.0) and rotate 0
            {2,1.0,0.0, 90.0},     // rotate 90
            {2,1.0,0.0, 180.0},    // rotate 180
            {2,1.0,0.0, 270.0},    // rotate 270
            {2,1.0,0.0, 360.0},    // rotate 360
            //{2,0.0,0.0, 0.0},

            {0,2.0,0.0, 0.0},      // go (2.0,0.0) and rotate 0
            {2,2.0,0.0, 90.0},     // rotate 90

            {0,2.0,0.4, 90.0},     // go (2.0,0.4) and rotate 90
            {2,2.0,0.4, 180.0},    // rotate 180
            {2,2.0,0.4, 270.0},    // rotate 270

            {0,2.0,0.0, 270.0},    // go (2.0,0.0) and rotate 270
            {2,2.0,0.0, -180.0},   // rotate -180

            {50,0.0,0.0, 0.0},     // set Navigation mode

            //{2,0.0,0.0, 0.0},
            //{99,0.0,0.0, 0.0},

            //{50,0.0,0.0, 0.0},

            {0,1.0,0.0, -180.0 },  // go (1.0,0.0) and rotate -180
            {2,1.0,0.0, 270.0 },   // rotate 270
            {2,1.0,0.0, 360.0},    // rotate 360
            {2,1.0,0.0, 90.0},     // rotate 90
            {2,1.0,0.0, 180.0},    // rotate 180
            //{2,0.0,0.0, 0.0},

            {0,0.0,0.0, 180.0},    // (0.0,0.0) and rotate 180
            {2,0.0,0.0, 270.0},    // rotate 270
            {2,0.0,0.0, 360.0},    // rotate 360
            {22,0.0,0.0, 0.0},
            {99,0.0,0.0, 0.0}       // end
            };

GoalList navi_list1[] ={
            {60, 0.0, 0.0, 0.0},    // course correct ON

            {10, 1.0, 0.0, 0},      // navi_move 1,0,0
            
            {2,1.0,0.0, 90.0},      // rotate 90
            {0,1.0,0.4, 90.0},      // go (2.0,0.4) and rotate 90

            //{50,0.0,0.0, 0.0},      // set Navigation mode
            //{22,0.0,0.0, 0.0},      // get map
            {99,0.0,0.0, 0.0},      // end
            };


GoalList2 m_goalList[] ={{60,0.0, 0.0},      // course correct ON
            {0, 0.0, 0},       // [0,0]
            {0, 0.0, 90},
            {0, 0.0, 180},
            {0, 0.0, 270},
            {0, 0.0, 360},
            //[2,0.0,0.0, 0.0],

            {0, 1.0, 0},        // [1,0]
            {0, 0.0, 90},
            {0, 0.0, 180},
            {0, 0.0, 270},
            {0, 0.0, 360},
            //[2,0.0,0.0, 0.0],

            {0, 1.0, 0},        // [2,0]
            {0, 0.0, 90},       // [2,0]

            {0, 0.4, 90},       // [2,0.4]
            {0,0.0, 180},
            {0,0.0, 270 },

            {0,0.4,270},         // [2,0]
            {0,0.0, -180},

            {50,0.0, 0.0},

            {60,0.0, 0.0},      // course correct ON
            //[62,0.0, 0.0],      // after_correct_wait ON
            //[99,0.0,0.0, 0.0],

            //[50,0.0,0.0, 0.0],

            {0,1.0, 180},     // [1.5,0]
            {0,0.0, 270},
            {0,0.0, 360},
            {0,0.0, 90},
            {0,0.0, 180},
            //[2,0.0,0.0, 0.0],

            {0,1.0, 180},      // [0,0]
            {0,0.0, 270},
            {0,0.0, 360},
            {22,0.0, 0.0},
            {99,0.0, 0.0}
            };


GoalList2 m_goalList2[] ={{0, 0.0, 0},       // [0,0]
            {0, 1.0, 0},        // [1,0]
            //{2,0.0,0.0, 0.0},
            {0, 1.0, 0},        // [2,0]
            {99, 0.0, 0}        // [3,0]
            };


GoalList2 m_goalList3[] ={{60, 0.0, 0.0},      // course correct ON
            {0, 0.0, 0},       // [0,0]
            {0, 0.0, 90},
            {0, 0.0, 180},
            {0, 0.0, 270},
            {0, 0.0, 360},
            //{2,0.0,0.0, 0.0},

            {0, 0.0, 90},        // [1,0]
            {0, 1.0, 90},
            {0, 0.0, 180},
            {0, 0.0, 270},

            {0, 2.0, 270},
            {0, 0.0, -180},
            {0, 0.0, -270},
            {0, 1.0, -270},

            //[2,0.0,0.0, 0.0],

            {0,0.0, -360},
            {99,0.0,0}
            };

// 1平米を動く 0.8 x 0.8
GoalList2 m_square_1[] ={{0, 0.0, 90},  // (0,0)
            {0, 0.4, 90},   // (0,0.4)
            {0, 0.0, -360},
            {0, 0.8, 0},     // (0.8,0.4)
            {0, 0.0, -90},
            {0, 0.2, -90},   // (0.8,0.2)
            {0, 0.0, -180},
            {0, 0.8, -180},  // (0.0,0.2)
            {0, 0.0, 270}, 
            {0, 0.2, 270},   // (0.0,0.0)
            //{0, 0.0, 0},   // 
            //{0, 0.8, 0},   //  (0.8,0.0)
            //{0, 0.0, -90},  
            //{0, 0.2, -90},   // (0.8,-0.2)
            //{0, 0.0, -180},
            //{0, 0.8, -180},  // (0.0,-0.2)
            //{0, 0.0, 270},   
            //{0, 0.2, 270},   // (0.0,-0.4)
            //{0, 0.0, 0},   
            //{0, 0.8, 0},   // (0.8,-0.4)
            {99, 0.0, 0}   // (0.0,0.0)
            };

GoalList2 m_rotate_1[] ={{0, 0, 0},
            {0, 0, 45},
            {0, 0, 90},  // (0,0)
            {0, 0, 180},
            {0, 0, 270},
            {0, 0, 360},
            {99, 0, 0}
            //[0, 1.0, -90],   // (0,0.4)
            };


GoalList2 m_rotate_2[] ={{0, 0, -0},
            {0, 0, -90},  // (0,0)
            {0, 0, -180},
            {0, 0, -270},
            {0, 0, -360},
            {99, 0, 0}
            };


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
    double x_size,y_size; 

public:

    GetMap(){}

    void init(ros::NodeHandle &nh,std::string map_frame="map")
    {
        nh_ = nh;
        _map_frame = map_frame;

        //_sub = nh.subscribe(_map_frame, 1);
        //self.map_info = None
        //self.map_data = None
        //self.grid = None
        //self.resolution = None
        _line_w = 8;  // ラインの幅 -> grid size [dot]  0.05[m] * 8 = 40[cm]
        _car_r = 2;    // ロボットの回転半径 [dot]
        _match_rviz = true;      // True / Flase  -> Rviz の画像と同じにする / しない
    }
 
    /*
    * https://answers.ros.org/question/293890/how-to-use-waitformessage-properly/
    * http://docs.ros.org/en/lunar/api/nav_msgs/html/msg/OccupancyGrid.html
    */
    void get(){
        int cnt=3;
        boost::shared_ptr<const nav_msgs::OccupancyGrid_<std::allocator<void>>> map_msg=nullptr;

        while (map_msg==nullptr && cnt >0){
            // auto map_msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map",nh_,ros::Duration(1.0));
            //printf("%s",map_msg);  // コンパイルエラーで、型が判る
            //boost::shared_ptr<const nav_msgs::OccupancyGrid_<std::allocator<void>>>

            map_msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(_map_frame,nh_,ros::Duration(1.0));
            //std::cout << "map_msg->header" << map_msg->header << std::endl; 
            //std::cout << "map_msg->info" << map_msg->info << std::endl;
            cnt--;
        }
        
        if (map_msg != nullptr){
            map_info = map_msg->info;
            std::cout << "map_info=" << std::endl;
            std::cout << map_info << std::endl;
            free_thresh = int(0.196 * 255);
            resolution = map_info.resolution;
            std::cout << "resolution=" << resolution << std::endl;

            org_x = map_info.origin.position.x;
            org_y = map_info.origin.position.y;
            std::cout << "org_x=" << org_x << std::endl;
            std::cout << "org_y=" << org_y << std::endl;

            x_size = 421 * resolution;
            y_size = 421 * resolution;

            std::cout << "x_size,y_size=" << x_size <<"," << y_size << std::endl;

            //print 'map_msg.data[0:10]=',map_msg.data[1:10]
            std::cout << "free_thresh=" << free_thresh << std::endl;
            conv_fmt2(map_msg);
            std::cout << "ok" << std::endl;
        }
        else{
            std::cout << "error" << std::endl;
        }
    }

    /*
    * conv_fmt2(nav_msgs::OccupancyGrid_ map_msg)
    */
    void conv_fmt2(boost::shared_ptr<const nav_msgs::OccupancyGrid_<std::allocator<void>>> map_msg){
        
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

        uint32_t height = int(map_msg->info.height / _line_w);
        uint32_t width = int(map_msg->info.width / _line_w);

        std::cout << "map_msg.info.height,map_msg.info.width=" << map_msg->info.height<<"," <<map_msg->info.width << std::endl;
        std::cout << "height,width=" << height << "," << width << std::endl;

        //self.grid = cv2.resize(m_grid,(width,height)) // resize map_msg.data
        //self.grid = self.np_resize(m_grid,(width,height));

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
    void conv_dot2grid(self,x,y){
        xi= int(x / self.resolution);
        yi= int(y/ self.resolution);
        gx = int(xi / self.line_w);
        gy = int(yi / self.line_w);
        return gx,gy;
    }
    /*
    convert Grid to Meter World
    */
    void conv_grid2meter(self,gx,gy,f_or_l){
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
    void conv_grid2meter_m_p(self,gx,gy,f_or_l){
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
    void conv_grid2meter_m_m(self,gx,gy,f_or_l){
        //x = (-210 + (gx * self.line_w + self.line_w/2)) * self.resolution
        x = (gx * self.line_w + self.line_w/2) * self.resolution + self.org_x   // chnaged by nishi 2020.12.2
        //y = (-210 + (gy * self.line_w + self.line_w/2)) * self.resolution
        y = (gy * self.line_w + self.line_w/2) * self.resolution + self.org_y   // chnaged by nishi 2020.12.2
        return y,x
    }

    void get_rotation_matrix(self,rad){
        /*
        指定したradの回転行列を返す
        */
        rot = np.array([[np.cos(rad), -np.sin(rad)],
                        [np.sin(rad), np.cos(rad)]]);
        return rot;
    }
    void np_resize(self,m_grid,req_size){
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
};

/*
* class MultiGoals
*/
class MultiGoals
{
private:
    int sts;
    int goalId;

    u_int8_t t_type;

    //! The node handle we'll be using
    ros::NodeHandle nh_;

    GetMap get_map;

    GoalList *_goalList;
    GoalList2 *_goalList2;

    RobotDrive drive;
    RobotNavi navi;

public:

    MultiGoals(){}

    //void init( map_frame,get_map,use_sim_time){
    void init(ros::NodeHandle &nh){
        nh_=nh;
        navi.init(nh,2);
        drive.init(nh);
        get_map.init(nh);

        goalId = 0;
        sts=0;
        t_type=0;

        sleep(1);
    }
    /*
    mloop_ex(GoalList *goalList)
        goalList: ゴールリスト
    */
    void mloop_ex(GoalList *goalList){
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
    void mloop_ex2(GoalList2 *goalList2){
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
              21 -> sleep
              22 -> get map
              50 -> set Navigation mode
              99 -> end
        func,dist,d_yaw
        func: 0 -> move dist and rotate d_yaw
              21 -> sleep
              22 -> get map
              50 -> set Navigation mode
              60 -> course_correct ON
              61 -> course_correct OFF
              62 -> after_correct_wait ON
              63 -> after_correct_wait OFF
              99 -> end
    */
    void mloop(){
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
                case 99:
                    f = false;
                    break;
            }
            //time.sleep(1)
            goalId += 1;
        }
    }

    void mloop_sub(){
        sts=0;
        int r_ct =0;

        float x,y,d_yaw,dist;

        if (t_type == 0){
            x = _goalList[goalId].x;
            y = _goalList[goalId].y;
            d_yaw = _goalList[goalId].d_yaw;
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
                navi.move(x,y,d_yaw/RADIANS_F);
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
    void call_service(){
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
};


int main(int argc, char** argv)
{
    //init the ROS node
    ros::init(argc, argv, "muliti_goals4");
    ros::NodeHandle nh;
    //ros::NodeHandle nh("~");

    //rosparam
    //map_frame = rospy.get_param('~map_frame', 'map' )
    //nh.getParam("~map_frame",   map_frame);
    //use_sim_time=rospy.get_param('/use_sim_time')
    //nh.getParam("/use_sim_time",   use_sim_time);


    //GetMap mp;
    //mp.init(nh);
    //mp.get();

    //RobotNavi navi;
    //navi.init(nh);

    MultiGoals mg_ex;
    mg_ex.init(nh);

    ros::Rate rate(1);   //  1[Hz]
    //for(int i=0;i<10;i++){
    //    ros::spinOnce();
    //    rate.sleep();
    //}

    //mg_ex.mloop_ex(goallist);
    //mg_ex.mloop_ex2(m_rotate_2);

    mg_ex.mloop_ex(navi_list1);

    //ros::Rate rate(1);   //  1[Hz]
    rate.sleep();

    //std::exit(0);
    ros::spin();
    return 0;
    
}
