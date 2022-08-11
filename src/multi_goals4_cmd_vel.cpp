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

#include "turtlebot3_navi_my/multi_goals.h"

#include <nav_msgs/OccupancyGrid.h>
//from nav_msgs.msg import OccupancyGrid,Odometry

// https://progsennin.com/c-initstructarray/221/


std::string map_frame="map";
bool use_sim_time = false;



GoalList goallist[] ={
            {60, 0.0, 0.0, 0.0},      // course correct ON
            {64, 0.0, 0.0, 0.0},      // go curve ON
            {66, 0.0, 0.0, 0.0},      // force current position to map(0,0)
            {67, 0.0, 0.0, 0.0},      // set dumper ON
            {0, 0.0, 0.0, 0.0},   // go (0.0,0.0) and rotate 0
            {2, 0.0, 0.0, 90.0},  // rotate 90
            {2, 0.0, 0.0, 180.0},  // rotate 180
            {2, 0.0, 0.0, 270.0},  // rotate 270
            {2, 0.0, 0.0, 360},   // rotate 360

            //{50,0.0,0.0, 0.0},      // set Navigation mode
            //{22,0.0,0.0, 0.0},      // get map
            {99,0.0,0.0, 0.0},      // end

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

            //{50,0.0,0.0, 0.0},     // set Navigation mode

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


GoalList turtlebot3_house[] ={
            {60, 0.0, 0.0, 0.0},      // course correct ON
            {64, 0.0, 0.0, 0.0},      // go curve ON
            {66, 0.0, 0.0, 0.0},      // force current position to map(0,0)
            {67, 0.0, 0.0, 0.0},      // set dumper ON
            {0, 0.0, 0.0, 0.0},         // go (0.0,0.0) and rotate 0
            {2, 0.0, 0.0, 90.0},        // rotate 90
            {2, 0.0, 0.0, 180.0},       // rotate 180
            {2, 0.0, 0.0, 270.0},       // rotate 270
            {2, 0.0, 0.0, 360},         // rotate 360
            //{0,0.0, -1.5, 270.0},      // go (0.0,-1.5) and rotate 270

            {50,0.0,0.0, 0.0},          // set Navigation mode
            //{22,0.0,0.0, 0.0},        // get map
            //{99,0.0,0.0, 0.0},          // end

            {0,1.0, 0.0, 0.0},      // go (1.0,0.0) and rotate 0
            //{69,0.0, 0.0, 0.0},     // save local cost map
            //{99,0.0,0.0, 0.0},      // end
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

            //{50,0.0,0.0, 0.0},     // set Navigation mode

            //{2,0.0,0.0, 0.0},
            {99,0.0,0.0, 0.0},

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



GoalList goallist2[] ={
            {60, 0.0, 0.0, 0.0},      // course correct ON
            {64, 0.0, 0.0, 0.0},      // go curve ON
            {0, 0.0, 0.0, 0.0},   // go (0.0,0.0) and rotate 0
            {0,2.0,0.5, 0.0},      // go (2.0,0.5) and rotate 0
            {0,5.0,0.0, 0.0},      // go (5.0,0.0) and rotate 0

            //{50,0.0,0.0, 0.0},      // set Navigation mode
            //{22,0.0,0.0, 0.0},      // get map
            {99,0.0,0.0, 0.0},      // end

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

    mg_ex.mloop_ex(goallist);
    //mg_ex.mloop_ex(turtlebot3_house);
    //mg_ex.mloop_ex(goallist2);
    //mg_ex.mloop_ex2(m_rotate_2);

    //mg_ex.mloop_ex(navi_list1);

    //unsigned char cost = mg_ex.drive.navi_.check_cost(-3.0,1.0);
    //std::cout <<"cost = "<< std::hex << (unsigned int)cost << std::endl;

    //ros::Rate rate(1);   //  1[Hz]
    rate.sleep();

    //std::exit(0);
    ros::spin();
    return 0;
    
}
