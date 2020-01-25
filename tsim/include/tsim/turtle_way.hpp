//
// Created by ricojia on 1/24/20.
//

#ifndef INC_495_NAV_ALL_PROJECTS_TURTLE_WAY_H
#define INC_495_NAV_ALL_PROJECTS_TURTLE_WAY_H

#include "rigid2d/waypoints.hpp"
#include <ros/ros.h>
#include <vector>
#include <turtlesim/Pose.h>
#include <tsim/PoseError.h>
#include <geometry_msgs/Twist.h>
#include "turtlesim/SetPen.h"
#include "turtlesim/TeleportAbsolute.h"


/// \file
/// \brief A node that has the turtle follow a trajectory of user-specified waypoints.
//The robot has a maximum forward velocity of 0.5 m/s and rotational velocity 0.5 rad/s
///
/// PARAMETERS:
///     parameter_name (parameter_type): description of the parameter
/// PUBLISHES:
///     /pose_error (tsim/PoseError): The error between pose estimation from dead-reckoning and the real pose.
/// SUBSCRIBES:
///     turtle/pose (turtlesim/Pose): the actual pose of the turtle for computing the error from odometry.

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

    ros::Subscriber pose_sub;
    ros::Publisher error_pub;
    ros::Publisher vel_pub;

    /// \brief  Upon receiving a new pose message from turtlesim, the function will publish error between the actual pose on turtlesim and pose that the robot keeps.
    /// \paramp pose(turtlesim::Pose) pose message from turtlesim
    void sub_callback(const turtlesim::Pose&);

    /// \brief turn pen on and off on turtlesim
    /// \param nh (ros::NodeHandle) public node handle of the robot
    /// \param status(bool): true for turning pen off
    void turn_off_pen( ros::NodeHandle& nh, bool);

    /// \brief Teleport turtle to the first element of the waypoints vector.
    /// \param nh (ros::Nodehandle) Public nodehandle
    /// \param init_heading(double) initial heading of the robot
    void teleport_2_bottom(ros::NodeHandle& , double);
};


#endif //INC_495_NAV_ALL_PROJECTS_TURTLE_WAY_H
