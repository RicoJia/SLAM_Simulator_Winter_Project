//
// Created by ricojia on 1/8/20.
//

#ifndef TSIM_TURTLE_RECT_H
#define TSIM_TURTLE_RECT_H

#include "ros/ros.h"
#include <sstream>
#include "turtle_rect.h"
#include "turtlesim/SetPen.h"
#include "turtlesim/TeleportAbsolute.h"
#include "geometry_msgs/Twist.h"
#include "tsim/traj_reset.h"
#include "tsim/PoseError.h"
#include "std_srvs/Empty.h"
#include "turtlesim/Pose.h"
#include "math.h"

void drop_pen(ros::NodeHandle nh, bool);

void teleport_2_bottom(ros::NodeHandle nh, double x, double y);
enum class State{
    go_forward,
    turn
};
void update_linear_pos(double v, double dt);
void update_angular_pos(double w, double dt);

bool traj_reset_callback();
void pose_callback(const turtlesim::Pose& pose_msg);
#endif //TSIM_TURTLE_RECT_H
