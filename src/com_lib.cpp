/*
* com_lib.cpp
*
*/

#include "turtlebot3_navi_my/com_lib.h"

void GetTF::init(ros::NodeHandle &nh){
    nh_=nh;

    //wait for the listener to get the first message
    //listener_.waitForTransform("base_footprint","odom", ros::Time(0), ros::Duration(1.0));
    //listener_.waitForTransform("base_footprint","map", ros::Time(0), ros::Duration(1.0));

    listener_.waitForTransform("map","base_footprint", ros::Time(0), ros::Duration(1.0));
    sleep(1);
}

void GetTF::get(int func){
    ros::Rate rate(70.0);
    while (1)
    {
        try
        {
            //listener_.lookupTransform("base_footprint","odom", ros::Time(0), base_tf);
            //listener_.lookupTransform("base_footprint","map", ros::Time(0), base_tf);
            listener_.lookupTransform("map","base_footprint", ros::Time(0), base_tf);
        }
        catch (tf::TransformException ex)
        {
            //ROS_ERROR("%s",ex.what());
            rate.sleep();
            continue;
            //break;
        }
        break;
    }

    if (func==2){
        //Quaternion getRotation()
        tf::Quaternion q = base_tf.getRotation();

        tf::Matrix3x3 m(q);
        //double roll, pitch, yaw;
        m.getRPY(_rx, _ry, _rz);
        if(log_level>=3)
            std::cout << "_rx: " << _rx << ", _ry: " << _ry << ", _rz: " << _rz << std::endl;
    }
}