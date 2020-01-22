//
// Created by ricojia on 1/20/20.
//

//#include "rigid2d/waypoints.hpp"
#include "../include/rigid2d/waypoints.hpp"
#include <math.h>

using namespace rigid2d;

double Waypoints::get_theta_error(){
    double theta = current_position.theta;
    auto target_vec = waypoints_vec[0] - Vector2D(current_position.x, current_position.y);
    double theta_required = angle(target_vec);
    double theta_err = normalize_angle(theta_required - theta);
    return theta_err;
}

double Waypoints::get_distance_error() {
    Vector2D current_coord(current_position.x, current_position.y);
    double distance_error = length(current_coord-waypoints_vec[0]);
    return distance_error;
}

void Waypoints::update_target(){
    waypoints_vec.push_back(waypoints_vec[0]);
    waypoints_vec.erase(waypoints_vec.begin());
}

void Waypoints::update_current_position(const Twist2D& velocities){
    current_position.x += velocities.x*cos(current_position.theta);
    current_position.y += velocities.y*sin(current_position.theta);
    current_position.theta = normalize_angle(current_position.theta + velocities.theta);
}

Twist2D Waypoints::nextWaypoint(const Twist2D& current_pose){
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
    update_current_position(cmd_vel);
    return cmd_vel;
}



bool Waypoints::reset_waypoints(const double& init_heading, const std::vector<Vector2D>& wps){
    current_position = Twist2D(init_heading, wps[0].x, wps[0].y);
    waypoints_vec = wps;
    update_target();
    return true;
}
