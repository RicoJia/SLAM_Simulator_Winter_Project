//
// Created by ricojia on 1/21/20.

/// \brief Odometry node that publishes messages on both nav_msgs/Odometry and tf tree.
/// A transform publishes a transform, and a Odometry msg publishes velocity.
/// It will keep track of the robot's position, based on the joint_state message it receives.
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
///   joint_states_odom (/sensor_msgs/JointState): topic to publish joint states on Rviz

#include <ros/ros.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include "rigid2d/diff_drive.hpp"

using std::string;
using namespace rigid2d;

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

WheelPos get_new_wheel_pos(WheelPos original_wheel_pos, const WheelVel& noisy_wheel_vel){
    original_wheel_pos.theta_l += noisy_wheel_vel.u_l;
    original_wheel_pos.theta_r += noisy_wheel_vel.u_r ;
    return original_wheel_pos;
}

Odometer::Odometer(){}
Odometer::Odometer(ros::NodeHandle& nh, ros::NodeHandle& nh2):diff_drive()
{
    nh.getParam("/Odometer/wheel_base", wheel_base);
    nh.getParam("/Odometer/wheel_radius", wheel_radius);
    nh.getParam("/Odometer/frequency", frequency);

    nh2.getParam("body_frame_id", body_frame_id);
    nh2.getParam("right_wheel_joint", right_wheel_joint);
    nh2.getParam("left_wheel_joint",left_wheel_joint);
    nh2.getParam("odom_frame_id",odom_frame_id);

    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
    current_time = ros::Time::now();
    sub = nh.subscribe("/joint_states", 10, &Odometer::sub_callback, this);

    auto init_pose = Twist2D();  //default pose
    diff_drive = DiffDrive(init_pose, wheel_base, wheel_radius);
}

void Odometer::sub_callback(const sensor_msgs::JointState& msg){
    // update odom here
    auto left_iterator = std::find(msg.name.begin(),msg.name.end(), left_wheel_joint);
    int left_index = std::distance(msg.name.begin(), left_iterator);
    auto right_iterator = std::find(msg.name.begin(),msg.name.end(), right_wheel_joint);
    int right_index = std::distance(msg.name.begin(), right_iterator);

    WheelVel wheel_vel_odometer(msg.velocity[left_index], msg.velocity[right_index]);

    //Update wheel positions, based on the noisy wheel velocities
    auto wheel_pos = diff_drive.wheelPositions();        //actual wheel_positions, after the noisy increment
    wheel_pos = get_new_wheel_pos(wheel_pos, wheel_vel_odometer);    //this is not updating the diff_drive parameter. This is for update odometry only.

    diff_drive.updateOdometry(wheel_pos.theta_l, wheel_pos.theta_r);    //update the real world twist of the robot.

    auto pose_twist = diff_drive.get_pose();
    diff_drive.update_wheel_pos(wheel_vel_odometer);
    auto velocity_twist = diff_drive.wheelsToTwist(wheel_vel_odometer);

    //construct odom msg and publish here
    nav_msgs::Odometry odom_msg = construct_odom_msg(pose_twist, velocity_twist);
    odom_pub.publish(odom_msg);

    //broadcast here
    geometry_msgs::TransformStamped odom_trans = construct_tf(pose_twist);
    odom_broadcaster.sendTransform(odom_trans);

    std::cout<<"------------"<<std::endl<<pose_twist<<std::endl;
}

nav_msgs::Odometry Odometer::construct_odom_msg(const rigid2d::Twist2D& pose_twist,const rigid2d::Twist2D& velocity_twist ){
    nav_msgs::Odometry odom_msg;
    current_time = ros::Time::now();
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = odom_frame_id;
    odom_msg.child_frame_id = body_frame_id;

    odom_msg.pose.pose.position.x = pose_twist.x;
    odom_msg.pose.pose.position.y = pose_twist.y;
    odom_msg.pose.pose.position.z = 0.0;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pose_twist.theta);
    odom_msg.pose.pose.orientation = odom_quat;

    odom_msg.twist.twist.linear.x = velocity_twist.x;
    odom_msg.twist.twist.linear.y = velocity_twist.y;
    odom_msg.twist.twist.angular.z = velocity_twist.theta;
    return odom_msg;
}

geometry_msgs::TransformStamped Odometer::construct_tf(const rigid2d::Twist2D& pose_twist){
    geometry_msgs::TransformStamped odom_trans;
    current_time = ros::Time::now();
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = odom_frame_id;
    odom_trans.child_frame_id = body_frame_id;

    odom_trans.transform.translation.x = pose_twist.x;
    odom_trans.transform.translation.y = pose_twist.y;
    odom_trans.transform.translation.z = 0.0;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pose_twist.theta);
    odom_trans.transform.rotation = odom_quat;
    return odom_trans;
}


int main(int argc, char** argv){
    ros::init(argc, argv, "Odometer");
    ros::NodeHandle nh;
    ros::NodeHandle nh2("~");
    Odometer odometer(nh,nh2);
    ros::spin();
    return 0;
}
