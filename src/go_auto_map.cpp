/*
go_auto_map.cpp

1. build
$ colcon build --symlink-install --parallel-workers 1 --packages-select turtlebot3_navi_my
$ . install/setup.bash

2. run
 $ ros2 launch turtlebot3_navi_my go_auto_map.launch.py use_sim_time:=True

*/

#include <iostream>
#include <unistd.h>
#include "turtlebot3_navi_my/pro_control_map.hpp"

#define USE_1LOOP

// Auto Map I ( nav2 and cmd_vel mode)
GoalList turtlebot3_auto_map[] ={
            {60, 0.0, 0.0, 0.0},      // course correct ON
            {64, 0.0, 0.0, 0.0},      // go curve ON
            {66, 0.0, 0.0, 0.0},      // force current position to map(0,0)
            //{67, 0.0, 0.0, 0.0},      // set dumper ON
            // 障害物からの距離の調整
            {73, 6.0, 0.0, 0.0},      // set set robo_r_     waffle 281 x 306[mm]    30.6/5 = 6.12 -> 7 / 2 -> 4
            {0, 0.0, 0.0, 0.0},       // go (0.0,0.0) and rotate 0
            {2, 0.0, 0.0, 90.0},      // rotate 90
            {2, 0.0, 0.0, 180.0},     // rotate 180
            {2, 0.0, 0.0, 270.0},     // rotate 270
            {2, 0.0, 0.0, 0.0},       // rotate 360
            //{10,0.0,1.0,90.0},         // navi move
            //{10,0.0,-1.0,-90.0},         // navi move
            {30,0.0,0.0, 0.0},        // Auto map builder
            {99,0.0,0.0, 0.0},        // end
};


// Auto map II ( nav2 and cmd_vel mode)
GoalList turtlebot3_auto_map_achor[] ={
            {60, 0.0, 0.0, 0.0},      // course correct ON
            {64, 0.0, 0.0, 0.0},      // go curve ON
            //{67, 0.0, 0.0, 0.0},      // set dumper ON
            // 障害物からの距離の調整
            {73, 6.0, 0.0, 0.0},      // set set robo_r_     waffle 281 x 306[mm]    30.6/5 = 6.12 -> 7 / 2 -> 4
            {66, 0.0, 0.0, 0.0},      // force current position to map(0,0)
            {0, 0.0, 0.0, 0.0},       // go (0.0,0.0) and rotate 0
            {2, 0.0, 0.0, 90.0},      // rotate 90
            {2, 0.0, 0.0, 180.0},     // rotate 180
            {2, 0.0, 0.0, 270.0},     // rotate 270
            {2, 0.0, 0.0, 0.0},       // rotate 360
            //{10,0.0,1.0,90.0},         // navi move
            //{10,0.0,-1.0,-90.0},         // navi move
            {31,0.0,0.0, 0.0},        // Auto map builder of anchor
            {99,0.0,0.0, 0.0},        // end
};

int main(int argc, char **argv){
    using namespace std::chrono_literals;
    rclcpp::WallRate loop(1);

    std::cout << "start " << std::endl;

    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("go_auto_map",rclcpp::NodeOptions{});

    loop.sleep();

    ProControlMap mg_ex;
    mg_ex.init(node);

    //mg_ex.mloop_ex(turtlebot3_auto_map);
    mg_ex.mloop_ex(turtlebot3_auto_map_achor);

    //ros::Rate rate(1);   //  1[Hz]
    while(rclcpp::ok()){
        //rclcpp::spin(std::make_shared<RobotDrive>());
        //get_tf.get(2);
        std::cout << "loop" << std::endl;
        loop.sleep();
    }
    //ros::spinOnce();

    //ros::spin();
    rclcpp::shutdown();

    return 0;
}
