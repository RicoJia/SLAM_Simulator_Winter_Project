//
// Created by ricojia on 1/23/20.
//

#include "rigid2d/fake_diff_encoders.hpp"

using namespace rigid2d;

FakeDiffEncoders::FakeDiffEncoders(ros::NodeHandle& nh, ros::NodeHandle& nh2):dd(){

    nh.getParam("/Fake_Diff_Encoders/wheel_base", wheel_base);
    nh.getParam("/Fake_Diff_Encoders/wheel_radius", wheel_radius);
    nh.getParam("/Fake_Diff_Encoders/frequency", frequency);

    nh2.getParam("right_wheel_joint", right_wheel_joint);
    nh2.getParam("left_wheel_joint",left_wheel_joint);
    joint_pub = nh.advertise< sensor_msgs::JointState>("joint_states_pure", 50);
    sub = nh.subscribe("cmd_vel", 10, &FakeDiffEncoders::sub_callback, this);
    dd = DiffDrive(0.0, 0.0, 0.0, wheel_base, wheel_radius);
}

void FakeDiffEncoders::sub_callback(const geometry_msgs::Twist& cmd_vel_msg){
    Twist2D body_twist(cmd_vel_msg.angular.z/frequency, cmd_vel_msg.linear.x/frequency, cmd_vel_msg.linear.y/frequency);
    auto wheel_vel = dd.twistToWheels(body_twist);
    auto wheel_pos = dd.update_wheel_pos(wheel_vel);
    auto joint_state_msg = construct_joint_state_msg(wheel_pos, wheel_vel);
    joint_pub.publish(joint_state_msg);
}

sensor_msgs::JointState FakeDiffEncoders::construct_joint_state_msg(const rigid2d::WheelPos& wheel_pos, const rigid2d::WheelVel& wheel_vel){
    sensor_msgs::JointState joint_state_msg;
    joint_state_msg.header.stamp = ros::Time::now();
    joint_state_msg.name = {left_wheel_joint, right_wheel_joint};
    joint_state_msg.position = {wheel_pos.theta_l, wheel_pos.theta_r};
    joint_state_msg.velocity = {wheel_vel.u_l, wheel_vel.u_r};
    return joint_state_msg;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "Fake_Diff_Encoder");
    ros::NodeHandle nh;
    ros::NodeHandle nh2("~");
    FakeDiffEncoders fake_diff_encoder(nh,nh2);
    ros::spin();
    return 0;
}