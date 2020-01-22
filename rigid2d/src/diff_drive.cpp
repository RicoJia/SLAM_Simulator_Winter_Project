//
// Created by ricojia on 1/19/20.
//

#include "rigid2d/diff_drive.hpp"
//#include "../include/rigid2d/diff_drive.hpp"
using Eigen::Matrix2d;
using Eigen::MatrixXd;
using Eigen::Vector2d;

rigid2d::WheelVel rigid2d::DiffDrive::twistToWheels(const Twist2D& body_twist){
    WheelVel wheelVel;
    try{
        if (body_twist.y != 0){
            throw std::logic_error("Body twist's y velocity should always be zero!");
        }
        else{
            Matrix2d H;
            H<<-1*(wheel_radius/wheel_base), (wheel_radius/wheel_base),
                    wheel_radius/2.0, wheel_radius/2.0;
            Vector2d v(body_twist.theta, body_twist.x);
            auto u = H.inverse()*v;
            wheelVel.u_l = u[0];
            wheelVel.u_r = u[1];

        }
    }
    catch (std::exception &e){ std::cerr<<"Error! "<<e.what()<<std::endl; }
    return wheelVel;
}

rigid2d::Twist2D rigid2d::DiffDrive::wheelsToTwist(const rigid2d::WheelVel& wheelVel){
    MatrixXd H(3,2);
    H(0,0) = -1*(wheel_radius/wheel_base); H(0,1) = (wheel_radius/wheel_base);
    H(1,0) = wheel_radius/2.0; H(1,1) = wheel_radius/2.0;
    H(2,0) = 0.0;   H(2,1) = 0.0;
    Vector2d u(wheelVel.u_l, wheelVel.u_r);
    auto v = H*u;
    Twist2D body_twist(v[0], v[1], v[2]);
    return body_twist;
}

rigid2d::WheelVel rigid2d::DiffDrive::updateOdometry(const double& l_encoding, const double& r_encoding){
    auto l_vel = l_encoding - wheel_positions.theta_l;
    auto r_vel = r_encoding - wheel_positions.theta_r;
    wheel_velocities.u_l = l_vel;
    wheel_velocities.u_r = r_vel;
    wheel_positions.theta_l = l_encoding;
    wheel_positions.theta_r = r_encoding;
    return wheel_velocities;
}

void rigid2d::DiffDrive::feedforward(const rigid2d::Twist2D& body_twist){
    pose.x += body_twist.x* cos(pose.theta);
    pose.y += body_twist.y* sin(pose.theta);
    pose.theta = normalize_angle(pose.theta + body_twist.theta);
}

rigid2d::WheelVel rigid2d::DiffDrive::wheelVelocities() const{
    return wheel_velocities;
}

void rigid2d::DiffDrive::reset(rigid2d::Twist2D ps){
    pose.theta = ps.theta; pose.x = ps.x; pose.y = ps.y;
    wheel_velocities.u_l = 0.0; wheel_velocities.u_r = 0.0;
    wheel_positions.theta_l = 0.0;wheel_positions.theta_r = 0.0;
}

rigid2d::Twist2D rigid2d::DiffDrive::get_pose(){
    return pose;
}