/*
* coom_lib.h
*
*/

#ifndef COM_LIB_H
#define COM_LIB_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>


#include <unistd.h>

#include "com_def.h"

/*
* T round_my(T dt,int n)
*/
template <class T=float>
T round_my(T dt,int n)
{
    if (n > 0){
        T x=(T)pow(10.0, n);
        dt *= x;
        return std::round(dt) / x;
    }
    else{
        return std::round(dt);
    }
}


class GetTF
{
private:
    //! The node handle we'll be using
    ros::NodeHandle nh_;
    //! We will be listening to TF transforms as well
    tf::TransformListener listener_;
    //geometry_msgs::Twist _vel_msg;

    u_char log_level=1;

public:
    tf::StampedTransform base_tf;

    double _rx, _ry, _rz;

    //! ROS node initialization
    GetTF(){}

    //GetTF(const GetTF& rgt){}
    //GetTF& operator=(const GetTF& rget);

    void init(ros::NodeHandle &nh);

    /*
    * T round_my(T dt,int n)
    */
    template <class T=float>
    T round_my(T dt,int n){
        if (n > 0){
            T x=(T)pow(10.0, n);
            dt *= x;
            return std::round(dt) / x;
        }
        else{
            return std::round(dt);
        }
    }
    /*
    * void get(int func)
    */
    void get(int func=0);
};

#endif  // COM_LIB