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
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include "std_srvs/Empty.h"
#include <unistd.h>

//#include <math.h>

#include "turtlebot3_navi_my/robot_drive.h"



// https://progsennin.com/c-initstructarray/221/

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

    GoalList *_goalList;
    GoalList2 *_goalList2;

    RobotDrive drive;

public:

    MultiGoals(){}

    //void init( map_frame,get_map,use_sim_time){
    void init(ros::NodeHandle &nh){
        nh_=nh;
        drive.init(nh);

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
                    mloop_sub();
                    break;
                case 21:
                    // sleep
                    sleep(1);
                    break;
                case 22:
                    //get_map.get();
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
        }
        else{
            dist = _goalList2[goalId].dist;
            d_yaw = _goalList2[goalId].d_yaw;
            drive.move(dist,d_yaw);
            //self.goalMsg.pose.position.y += dist * math.sin(math.radians(d_yaw));
            //self.goalMsg.pose.position.x += dist * math.cos(math.radians(d_yaw));
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

    MultiGoals mg_ex;
    mg_ex.init(nh);

    mg_ex.mloop_ex(goallist);

    //mg_ex.mloop_ex2(m_rotate_2);

    ros::Rate rate(1);   //  1[Hz]
    rate.sleep();

    std::exit(0);
}
