#include "ros/ros.h"
#include <sstream>
#include "turtle_rect.h"
#include "turtlesim/SetPen.h"
#include "turtlesim/TeleportAbsolute.h"
#include "geometry_msgs/Twist.h"

void drop_pen(ros::NodeHandle nh, bool status)
{
    ros::service::waitForService("/turtle1/set_pen", 100);
    ros::ServiceClient client = nh.serviceClient<turtlesim::SetPen>("/turtle1/set_pen");
    turtlesim::SetPen setpen_srv;
    setpen_srv.request.r = 255;        //can we use a constructor here?
    setpen_srv.request.g = 0;
    setpen_srv.request.b = 0;
    setpen_srv.request.width = 1;
    setpen_srv.request.off = status;

    client.call(setpen_srv);
}

void teleport_2_bottom(ros::NodeHandle nh, double x, double y)
{
    ros::service::waitForService("/turtle1/teleport_absolute", 10000);
    ros::ServiceClient teleport_client = nh.serviceClient<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute");
    turtlesim::TeleportAbsolute teleportabsolute_srv;
    teleportabsolute_srv.request.x = x;        //can we use a constructor here? Better way of doing this?
    teleportabsolute_srv.request.y = y;
    teleportabsolute_srv.request.theta = 0;

    teleport_client.call(teleportabsolute_srv);
}

int main(int argc, char**argv)
{
    ros::init(argc, argv,"turtle_rect");
    ros::NodeHandle nh;

    double x, y, width, height, trans_vel, rot_vel;
    int frequency;

    if (nh.getParam("/turtle_rect/x", x)){      //How to get rid of turtle_rect??
        ROS_INFO("x: %f", x);
    }

    if (nh.getParam("/turtle_rect/y", y)){
        ROS_INFO("y: %f", y);
    }
    if (nh.getParam("/turtle_rect/width", width)){
        ROS_INFO("width: %f", width);
    }
    if (nh.getParam("/turtle_rect/height", height)){
        ROS_INFO("height: %f", height);
    }
    if (nh.getParam("/turtle_rect/trans_vel", trans_vel)){
        ROS_INFO("trans_vel: %f",trans_vel);
    }
    if (nh.getParam("/turtle_rect/rot_vel", rot_vel)){
        ROS_INFO("rot_vel: %f",rot_vel);
    }
    if (nh.getParam("/turtle_rect/frequency", frequency)){
        ROS_INFO("frequency: %d",frequency);
    }

    drop_pen(nh, true);
    teleport_2_bottom(nh,x, y);
    drop_pen(nh, false);
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
    ros::Rate loop_rate(frequency);
    State turtle_state = State::go_forward;
    ros::Time init_time = ros::Time::now();
    double turn_time = 3.1415927/2.0/rot_vel;
    while (ros::ok){
        geometry_msgs::Twist msg;
        if (turtle_state == State ::go_forward){
            msg.linear.x = trans_vel;
            msg.angular.z = 0;
            if (ros::Time::now()-init_time>=ros::Duration(1)){
                turtle_state = State ::turn;
                init_time = ros::Time::now();
            }
        }
        else{
            msg.linear.x = 0;
            msg.angular.z = rot_vel;
            if (ros::Time::now() - init_time>= ros::Duration(turn_time)){
                turtle_state = State::go_forward;
                init_time = ros::Time::now();
            }
        }

        cmd_vel_pub.publish(msg);
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}