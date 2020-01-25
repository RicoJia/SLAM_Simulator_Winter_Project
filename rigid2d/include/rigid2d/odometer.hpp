//
// Created by ricojia on 1/21/20.
//

#ifndef INC_495_NAV_ALL_PROJECTS_ODOMETER_H
#define INC_495_NAV_ALL_PROJECTS_ODOMETER_H

/// \brief Odometry node that publishes messages on both nav_msgs/Odometry and tf tree.
/// A transform publishes a transform, and a Odometry msg publishes velocity.
/// Things to do:

/// PARAMETERS:
///    dd: DiffDrive object
///    wheel_base - distance between the two wheels
///    wheel_radius - radius of each wheel
///    frequency - The frequency of the control loop
/// PUBLISHES:  (after receiving subscribed topic updates)
///    odom (nav_msgs/Odometry): odometry message of the robot
/// BROADCASTS:
///    tf/transform_broadcaster: transform for Rviz
/// SUBSCRIBES:
///   joint_states (/sensor_msgs/JointState): topic to publish joint states on Rviz

#include <ros/ros.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include "../include/rigid2d/diff_drive.hpp"

using std::string;


class Odometer{
public:
    /// \brief Default constructor
    Odometer();

    /// \brief constructor
    explicit Odometer(ros::NodeHandle& nh, ros::NodeHandle& nh2);

private:
    double wheel_base, wheel_radius;
    int frequency;
    ros::Subscriber sub;
    ros::Publisher odom_pub;
    tf::TransformBroadcaster odom_broadcaster;
    ros::Time current_time;

    string odom_frame_id;
    string body_frame_id;
    string left_wheel_joint;
    string right_wheel_joint;
    rigid2d::DiffDrive diff_drive;

    /// \brief Callback function for receiving joint_state msg.
    /// \param sensor_msgs::JointState&  joint state messages for left and right wheel velocities
    void sub_callback(const sensor_msgs::JointState& msg);

    /// \brief constructing a odom message,  based on pose and body twist
    /// \param sensor_msgs::JointState&  joint state messages for left and right wheel velocities
    nav_msgs::Odometry construct_odom_msg(const rigid2d::Twist2D&, const rigid2d::Twist2D&);
    geometry_msgs::TransformStamped construct_tf(const rigid2d::Twist2D&);

};


#endif //INC_495_NAV_ALL_PROJECTS_ODOMETER_H
