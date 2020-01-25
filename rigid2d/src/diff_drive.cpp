//
// Created by ricojia on 1/19/20.
//

//#include "rigid2d/diff_drive.hpp"
#include "rigid2d/diff_drive.hpp"
using Eigen::Matrix2d;
using Eigen::MatrixXd;
using Eigen::Vector2d;

using namespace rigid2d;

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
    if (v[0] == -1*rigid2d::PI)
         body_twist.theta = PI;

    return body_twist;
}

rigid2d::WheelVel rigid2d::DiffDrive::updateOdometry(const double& l_encoding, const double& r_encoding){
    auto l_vel = normalize_angle( normalize_angle( l_encoding ) - wheel_positions.theta_l);
    auto r_vel = normalize_angle( normalize_angle( r_encoding ) - wheel_positions.theta_r);
    //if the |difference|>PI, according to our speed limit, we choose the increment that is less than PI.
    if (abs(l_vel)> PI)
        l_vel = -1.0*l_vel/abs(l_vel) * (PI - abs(l_vel));

    if (abs(r_vel)> PI)
        l_vel = -1.0*r_vel/abs(r_vel) * (PI - abs(r_vel));

    wheel_velocities.u_l = l_vel;
    wheel_velocities.u_r = r_vel;
    wheel_positions.theta_l = normalize_angle(l_encoding);
    wheel_positions.theta_r = normalize_angle(r_encoding);

    auto body_twist = wheelsToTwist(wheel_velocities);
    auto new_transform = integrateTwist(body_twist);
    pose_transform*=new_transform;
    return wheel_velocities;
}

void rigid2d::DiffDrive::feedforward(const rigid2d::Twist2D& body_twist){

    auto new_transform = integrateTwist(body_twist);
    pose_transform*=new_transform;
}

rigid2d::WheelVel rigid2d::DiffDrive::wheelVelocities() const{
    return wheel_velocities;
}

void rigid2d::DiffDrive::reset(rigid2d::Twist2D ps){
    pose_transform = Transform2D(Vector2D(ps.x, ps.y), ps.theta);
    wheel_velocities.u_l = 0.0; wheel_velocities.u_r = 0.0;
    wheel_positions.theta_l = 0.0;wheel_positions.theta_r = 0.0;
}

rigid2d::Twist2D rigid2d::DiffDrive::get_pose(){
    return pose_transform.displacement();
}

rigid2d::WheelPos rigid2d::DiffDrive::update_wheel_pos(const rigid2d::WheelVel& wheel_vel)
{
    wheel_positions.theta_r = rigid2d::normalize_angle(wheel_positions.theta_r + wheel_vel.u_r);
    wheel_positions.theta_l = rigid2d::normalize_angle(wheel_positions.theta_l + wheel_vel.u_l);
    return wheel_positions;
}
