//
// Created by ricojia on 1/23/20.
//

#ifndef INC_495_NAV_ALL_PROJECTS_FAKE_ENCODERS_H
#define INC_495_NAV_ALL_PROJECTS_FAKE_ENCODERS_H


#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <geometry_msgs/Twist.h>
#include "rigid2d/diff_drive.hpp"

using std::string;

/// \file
/// \brief The node subscribes to geometry_msgs/Twist messages on cmd_vel. Assume that, once a cmd_vel is received, the robot follows that velocity until the next cmd_vel command
/// PARAMETERS:
///     sensor_msgs::geometry_msgs::TransformStamped construct_tf(const rigid2d::Twist2D&);
///     frequency : frequency for updating cmd_vel. This is needed for determining how far the robot goes at a rate of cmd_vel.
/// SUBSCRIBES:
///     sensor_msgs::JointState&  joint state messages for left and right wheel velocities
/// PUBLISHES:
///     sensor_msgs::nav_msgs::Odometry construct_odom_msg(const rigid2d::Twist2D&, const rigid2d::Twist2D&);
/// BROADCAST:
///     /tf transformation between /odom frame and /base_link fram


class FakeDiffEncoders{
public:
    FakeDiffEncoders(ros::NodeHandle&, ros::NodeHandle&);

private:
    int frequency;
    double wheel_base, wheel_radius;
    string left_wheel_joint;
    string right_wheel_joint;
    ros::Subscriber sub;
    ros::Publisher joint_pub;
    rigid2d::DiffDrive dd;

    /// \brief Callback function for receiving cmd_vel msg.
    /// \param geometry_msgs::Twist& geometry_msg messages for left and right wheel velocities
    void sub_callback(const geometry_msgs::Twist& cmd_vel_msg);

    /// \brief Callback function for receiving cmd_vel msg.
    /// \param geometry_msgs::Twist& geometry_msg messages for left and right wheel velocities
    sensor_msgs::JointState construct_joint_state_msg(const rigid2d::WheelPos& wheel_pos, const rigid2d::WheelVel& wheel_vel);
};


#endif //INC_495_NAV_ALL_PROJECTS_FAKE_ENCODERS_H
