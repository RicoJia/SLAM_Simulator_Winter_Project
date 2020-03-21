//
// Created by ricojia on 1/20/20.
//

//#include "rigid2d/waypoints.hpp"
#include "rigid2d/waypoints.hpp"
#include <math.h>

using namespace rigid2d;

double Waypoints::get_theta_error(){
    double theta = dd.get_pose().theta;
    auto target_vec = waypoints_vec[0] - Vector2D(dd.get_pose().x, dd.get_pose().y);
    double theta_required = angle(target_vec);
    double theta_err = normalize_angle(theta_required - theta);
    return theta_err;
}

double Waypoints::get_distance_error() {
    Vector2D current_coord(dd.get_pose().x, dd.get_pose().y);
    double distance_error = length(current_coord-waypoints_vec[0]);
    return distance_error;
}

void Waypoints::update_target(){
    waypoints_vec.push_back(waypoints_vec[0]);
    waypoints_vec.erase(waypoints_vec.begin());
}

void Waypoints::update_current_position(const Twist2D& velocities){
    dd.feedforward(velocities);
}

Twist2D Waypoints::nextWaypoint(){
    double theta_err = get_theta_error();
    Twist2D cmd_vel;
    if (abs(theta_err) > WAYPOINTS_THETA_THRESHOLD){
        cmd_vel.theta = (abs(theta_err) >= abs(velocity_lim.theta))?theta_err/abs(theta_err)*velocity_lim.theta:theta_err;
    }
    else{
        double distance_err = get_distance_error();
        if (abs(distance_err) > WAYPOINTS_DISTANCE_THRESHOLD){
            cmd_vel.x = ( abs(distance_err) >= abs(velocity_lim.x)) ? abs(distance_err)/distance_err*velocity_lim.x: distance_err;
        }
        else{
            update_target();
        }
    }

    Twist2D cmd_increment(cmd_vel.theta/frequency, cmd_vel.x/frequency, cmd_vel.y/frequency);
    update_current_position(cmd_increment);
    return cmd_vel;
}

Twist2D Waypoints::get_pose(){
    return dd.get_pose();
}

void Waypoints::reset_waypoints(const double& init_heading, const std::vector<Vector2D>& wps){
    dd = DiffDrive(init_heading, wps[0].x, wps[0].y, 0.0, 0.0);
    waypoints_vec = wps;
    update_target();
}
