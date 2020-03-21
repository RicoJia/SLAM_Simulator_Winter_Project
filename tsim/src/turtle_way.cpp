/// \file
/// \brief A node that has the turtle follow a trajectory of user-specified waypoints.
//The robot has a maximum forward velocity of 0.5 m/s and rotational velocity 0.5 rad/s
///
/// PARAMETERS:
///     parameter_name (parameter_type): description of the parameter
/// PUBLISHES:
///     turtle1/cmd_vel (geometry_msgs::Twist): cmd_vel as the noiseless control input

#include "rigid2d/waypoints.hpp"
#include <ros/ros.h>
#include <vector>
#include <geometry_msgs/Twist.h>

using std::vector;
using namespace rigid2d;
class TurtleWay {
public:
    TurtleWay();
    TurtleWay(ros::NodeHandle& nh, ros::NodeHandle& nh2);
    void publish_velocity_commands();

    double frequency;
private:
    std::vector<rigid2d::Vector2D> wp_vec;
    double wheel_base;
    double wheel_radius;
    rigid2d::Twist2D max_vel;
    rigid2d::Waypoints wp;

    ros::Publisher vel_pub;
};



TurtleWay::TurtleWay(){}

TurtleWay::TurtleWay(ros::NodeHandle& nh, ros::NodeHandle& nh2)
{

    vector<double> waypoints_x, waypoints_y;
    nh.getParam("/TurtleWay/wheel_base", wheel_base);
    nh.getParam("/TurtleWay/wheel_radius", wheel_radius);
    nh.getParam("/TurtleWay/frequency", frequency);
    nh.getParam("/TurtleWay/waypoints_x", waypoints_x);
    nh.getParam("/TurtleWay/waypoints_y", waypoints_y);

    double rot_vel, trans_vel;
    nh2.getParam("rot_vel", rot_vel);
    nh2.getParam("trans_vel", trans_vel);
    max_vel = Twist2D(rot_vel, trans_vel, 0.0);

    for (unsigned i = 0; i < waypoints_x.size(); ++i){
        Vector2D waypoint(waypoints_x[i], waypoints_y[i]);
        wp_vec.push_back(waypoint);
    }

    double init_heading = 0.0;
    wp = Waypoints(init_heading, wheel_base, wheel_radius, wp_vec, max_vel, frequency);

    vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 50);
}


void TurtleWay::publish_velocity_commands() {
    auto commanded_vel = wp.nextWaypoint();
    geometry_msgs::Twist cmd_vel;
    cmd_vel.angular.z = commanded_vel.theta;
    cmd_vel.linear.x = commanded_vel.x;
    vel_pub.publish(cmd_vel);
}


int main(int argc, char** argv){
    ros::init(argc, argv, "TurtleWay");
    ros::NodeHandle nh;
    ros::NodeHandle nh2("~");
    TurtleWay turtleway(nh,nh2);
    ros::Rate rate( turtleway.frequency );
    while (ros::ok()){
        turtleway.publish_velocity_commands();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
