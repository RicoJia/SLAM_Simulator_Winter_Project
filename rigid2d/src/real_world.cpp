//
// Created by ricojia on 1/28/20.
//

/// \brief Node that gives simulates the real world pose of a diff drive robot, after adding noise to the wheels for slippage simulation. The node also determines the encoder
/// values of wheels, which will have a drift.
/// The real_world listens to only the velocity info from Fake Encoders, then it keeps track of the robot's wheel position using that information
/// Also, the real world will set the robot's initial position to (0, 0,0)

/// PARAMETERS:
///    dd: DiffDrive object
///    wheel_base - distance between the two wheels
///    wheel_radius - radius of each wheel
///    frequency - The frequency of the control loop
/// PUBLISHES:  (after receiving subscribed topic updates)
///    map (nav_msgs/Odometry): odometry message containing the real pose info of the robot in the map frame
/// BROADCASTS:
///    tf/transform_broadcaster: transform for Rviz
/// SUBSCRIBES:
///   joint_states_pure (/sensor_msgs/JointState): topic to publish joint states on Rviz


#include <ros/ros.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include "rigid2d/diff_drive.hpp"
#include <random>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>


using std::string;
using namespace rigid2d;


class RealWorld{
public:
    /// \brief Default constructor
    RealWorld();
    /// \brief constructor
    explicit RealWorld(ros::NodeHandle& nh, ros::NodeHandle& nh2);

private:
    double wheel_base, wheel_radius;
    int frequency;
    ros::Subscriber sub;
    ros::Publisher map_odom_pub;
    ros::Publisher joint_state_odom_pub;
    ros::Publisher joint_state_pub;
    tf::TransformBroadcaster map_odom_broadcaster;
    tf::TransformListener listener;
    ros::Time current_time;

    string map_frame_id;
    string odom_frame_id;
    string left_wheel_joint;
    string right_wheel_joint;
    rigid2d::DiffDrive diff_drive;
    double slip_miu, odom_miu, odom_drift;

    /// \brief Callback function for receiving joint_state msg.
    /// \param sensor_msgs::JointState&  joint state messages for left and right wheel velocities
    void sub_callback(const sensor_msgs::JointState& msg);
//
    /// \brief constructing a map_odom message,  based on pose and body twist
    /// \param sensor_msgs::JointState&  joint state messages for left and right wheel velocities
    geometry_msgs::TransformStamped construct_tf(const rigid2d::Twist2D&);

    nav_msgs::Odometry construct_map_odom_msg(const rigid2d::Twist2D&, const rigid2d::Twist2D&);

    sensor_msgs::JointState construct_joint_state_msg(const rigid2d::WheelPos& wheel_pos, const rigid2d::WheelVel& wheel_vel);
};

WheelPos get_new_wheel_pos(WheelPos original_wheel_pos, const WheelVel& noisy_wheel_vel){
    original_wheel_pos.theta_l += noisy_wheel_vel.u_l;
    original_wheel_pos.theta_r += noisy_wheel_vel.u_r ;
    return original_wheel_pos;
}

WheelVel add_noise(double u_l, double u_r, double mean, double miu){
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(mean,miu);
    double _u_l = u_l+ distribution(generator);
    double _u_r = u_r + distribution(generator);
    return  WheelVel(_u_l, _u_r);
}

RealWorld::RealWorld(){}
RealWorld::RealWorld(ros::NodeHandle& nh, ros::NodeHandle& nh2):diff_drive()
{
    nh.getParam("/RealWorld/wheel_base", wheel_base);
    nh.getParam("/RealWorld/wheel_radius", wheel_radius);
    nh.getParam("/RealWorld/frequency", frequency);

    nh2.getParam("odom_frame_id", odom_frame_id);
    nh2.getParam("right_wheel_joint", right_wheel_joint);
    nh2.getParam("left_wheel_joint",left_wheel_joint);
    nh2.getParam("map_frame_id",map_frame_id);

    nh2.getParam("slip_miu",slip_miu);
    nh2.getParam("odom_miu",odom_miu);
    nh2.getParam("odom_drift",odom_drift);

    map_odom_pub = nh.advertise<nav_msgs::Odometry>("map_odom", 50);
    joint_state_odom_pub = nh.advertise<sensor_msgs::JointState>("joint_states_odom", 50);
    joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 50);
    current_time = ros::Time::now();
    sub = nh.subscribe("/joint_states_pure", 10, &RealWorld::sub_callback, this);
    auto init_pose = Twist2D();  //default pose
    diff_drive = DiffDrive(init_pose, wheel_base, wheel_radius);
}



nav_msgs::Odometry RealWorld::construct_map_odom_msg(const rigid2d::Twist2D& pose_twist,const rigid2d::Twist2D& velocity_twist ){
    nav_msgs::Odometry odom_msg;
    auto current_time = ros::Time::now();
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = map_frame_id;
    odom_msg.child_frame_id = odom_frame_id;

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


/// \brief constructing a joint state message, based on the current wheel position(updated by wheel_vel), and wheel velocity
sensor_msgs::JointState RealWorld::construct_joint_state_msg(const rigid2d::WheelPos& wheel_pos, const rigid2d::WheelVel& wheel_vel){
    sensor_msgs::JointState joint_state_msg;
    joint_state_msg.header.stamp = ros::Time::now();
    joint_state_msg.name = {left_wheel_joint, right_wheel_joint};
    joint_state_msg.position = {wheel_pos.theta_l, wheel_pos.theta_r};
    joint_state_msg.velocity = {wheel_vel.u_l, wheel_vel.u_r};
    return joint_state_msg;
}


geometry_msgs::TransformStamped RealWorld::construct_tf(const rigid2d::Twist2D& pose_twist){
    geometry_msgs::TransformStamped odom_trans;
    tf::StampedTransform transform;
    current_time = ros::Time::now();
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = map_frame_id;
    odom_trans.child_frame_id = odom_frame_id;

    //calculate map to odom. if nothing is available, publish [0,0,0]
    try{
        listener.lookupTransform("base_link", "odom", ros::Time(0), transform);     //TODO: or reverse?

        tf::Quaternion q = transform.getRotation();
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        double x_bo,y_bo;
        x_bo = transform.getOrigin().x();
        y_bo = transform.getOrigin().y();

        Transform2D Tsb(Vector2D( pose_twist.x, pose_twist.y), pose_twist.theta);
        Transform2D Tbo(Vector2D(x_bo, y_bo),yaw);
        auto Tso = Tsb*Tbo;
        Twist2D Xso = Tso.displacement();

        odom_trans.transform.translation.x = Xso.x;
        odom_trans.transform.translation.y = Xso.y;
        odom_trans.transform.translation.z = 0.0;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(Xso.theta);
        odom_trans.transform.rotation = odom_quat;
    }
    catch (tf::TransformException &ex){
        odom_trans.transform.translation.x = 0.0;
        odom_trans.transform.translation.y = 0.0;
        odom_trans.transform.translation.z = 0.0;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0.0);
        odom_trans.transform.rotation = odom_quat;
    }

    return odom_trans;
}


void RealWorld::sub_callback(const sensor_msgs::JointState& msg){

    //broadcast transform here
    auto pose_twist = diff_drive.get_pose();
    geometry_msgs::TransformStamped map_odom_trans = construct_tf(pose_twist);
    map_odom_broadcaster.sendTransform(map_odom_trans);
    // update odom here

    auto left_iterator = std::find(msg.name.begin(),msg.name.end(), left_wheel_joint);
    int left_index = std::distance(msg.name.begin(), left_iterator);
    auto right_iterator = std::find(msg.name.begin(),msg.name.end(), right_wheel_joint);
    int right_index = std::distance(msg.name.begin(), right_iterator);

    // Adding noise to wheel veloctity, for the real world itself and for the odometer.
    auto wheel_vel_map = add_noise(msg.velocity[left_index], msg.velocity[right_index], 0, slip_miu);
    auto wheel_vel_odometer = add_noise(wheel_vel_map.u_l, wheel_vel_map.u_r, odom_drift, odom_miu);

    //Update wheel positions, based on the noisy wheel velocities
    auto wheel_pos = diff_drive.wheelPositions();        //actual wheel_positions, after the noisy increment
    wheel_pos = get_new_wheel_pos(wheel_pos, wheel_vel_map);    //this is not updating the diff_drive parameter. This is for update odometry only.

    diff_drive.updateOdometry(wheel_pos.theta_l, wheel_pos.theta_r);    //update the real world twist of the robot.
    diff_drive.update_wheel_pos(wheel_vel_map);

    auto velocity_twist_map = diff_drive.wheelsToTwist(wheel_vel_map);

    //construct odom msg and publish here, also, don't forget to add some noise in.
    nav_msgs::Odometry map_odom_msg = construct_map_odom_msg(pose_twist,velocity_twist_map);        //TODO: Cause problem with old pose_twist?
    map_odom_pub.publish(map_odom_msg);

    //construct joint state msgs for simulator and odometer
    sensor_msgs::JointState joint_state_msg = construct_joint_state_msg(wheel_pos,wheel_vel_map);
    sensor_msgs::JointState joint_state_odom_msg = construct_joint_state_msg(wheel_pos, wheel_vel_odometer);
    joint_state_pub.publish(joint_state_msg);
    joint_state_odom_pub.publish(joint_state_odom_msg);


}
//
//void RealWorld::sub_callback(const sensor_msgs::JointState& msg){
//    // update odom here
//    auto left_iterator = std::find(msg.name.begin(),msg.name.end(), left_wheel_joint);
//    int left_index = std::distance(msg.name.begin(), left_iterator);
//    auto right_iterator = std::find(msg.name.begin(),msg.name.end(), right_wheel_joint);
//    int right_index = std::distance(msg.name.begin(), right_iterator);
//
//    // Adding noise to wheel veloctity, for the real world itself and for the odometer.
//    auto wheel_vel_map = add_noise(msg.velocity[left_index], msg.velocity[right_index], 0, slip_miu);
//    auto wheel_vel_odometer = add_noise(wheel_vel_map.u_l, wheel_vel_map.u_r, odom_drift, odom_miu);
//
//    //Update wheel positions, based on the noisy wheel velocities
//    auto wheel_pos = diff_drive.wheelPositions();        //actual wheel_positions, after the noisy increment
//    wheel_pos = get_new_wheel_pos(wheel_pos, wheel_vel_map);    //this is not updating the diff_drive parameter. This is for update odometry only.
//
//    diff_drive.updateOdometry(wheel_pos.theta_l, wheel_pos.theta_r);    //update the real world twist of the robot.
//    auto pose_twist = diff_drive.get_pose();
//    diff_drive.update_wheel_pos(wheel_vel_map);
//
//    auto velocity_twist_map = diff_drive.wheelsToTwist(wheel_vel_map);
//
//    //construct odom msg and publish here, also, don't forget to add some noise in.
//    nav_msgs::Odometry map_odom_msg = construct_map_odom_msg(pose_twist,velocity_twist_map);
//    map_odom_pub.publish(map_odom_msg);
//
//    //construct joint state msgs for simulator and odometer
//    sensor_msgs::JointState joint_state_msg = construct_joint_state_msg(wheel_pos,wheel_vel_map);
//    sensor_msgs::JointState joint_state_odom_msg = construct_joint_state_msg(wheel_pos, wheel_vel_odometer);
//    joint_state_pub.publish(joint_state_msg);
//    joint_state_odom_pub.publish(joint_state_odom_msg);
//
//    //broadcast here
//    geometry_msgs::TransformStamped map_odom_trans = construct_tf(pose_twist);
//    map_odom_broadcaster.sendTransform(map_odom_trans);
//
//}

int main(int argc, char**argv){
    ros::init(argc, argv, "RealWorld");
    ros::NodeHandle nh;
    ros::NodeHandle nh2("~");
    RealWorld real_world(nh,nh2);
    ros::spin();
    return 0;
}