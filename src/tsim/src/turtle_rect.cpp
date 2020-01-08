#include "ros/ros.h"
#include <sstream>

int main(int argc, char**argv)
{
    ros::init(argc, argv,"turtle_rect");
    ros::NodeHandle nh;
    double x, y, width, height, trans_vel, rot_vel;
    int frequency;
    while (ros::ok){
        if (nh.getParam("/turtle_rect/x", x)){      //How to get rid of turtle_rect??
            ROS_INFO("x: %f", x);
        }

        if (nh.getParam("/turtle_rect/y", y)){
            ROS_INFO("y: %f", y);
        }
        if (nh.getParam("/turtle_rect/y", y)){
            ROS_INFO("y: %f", y);
        }
        if (nh.getParam("/turtle_rect/y", y)){
            ROS_INFO("y: %f", y);
        }

    }

    ros::spin();
    return 0;
}