//
// Created by ricojia on 1/24/20.
//

#ifndef INC_495_NAV_ALL_PROJECTS_TURTLE_WAY_H
#define INC_495_NAV_ALL_PROJECTS_TURTLE_WAY_H

#include "rigid2d/waypoints.hpp"

/// \file
/// \brief A node that has the turtle follow a trajectory of user-specified waypoints.
//The robot has a maximum forward velocity of 0.5 m/s and rotational velocity 0.5 rad/s
///
/// PARAMETERS:
///     parameter_name (parameter_type): description of the parameter
/// PUBLISHES:
///     /pose_error (tsim/PoseError): The error between pose estimation from dead-reckoning and the real pose.
/// SUBSCRIBES:
///     turtle/pose (turtlesim/Pose): the actual pose of the turtle for computing the error from odometry.

The waypoint y coordinates are specified in the waypoint_y parameter
The node should also subscribe to  and compute the error between its actual and expected trajectory
        For this task, the trajectory must be computed without knowledge of the pose.
The error should be published on /pose_error
        As in turtle_rect, the turtle should pick its pen up, teleport to the first waypoint, and put its pen down and
        The robot can publish velocity commands at a fixed rate of 60Hz

class TurtleWay {
public:
    TurtleWay();
private:
    std::vector<rigid2d::Vector2D> wp_vec;
    unsigned int frequency;
    rigid2d::Waypoints wp;
};


#endif //INC_495_NAV_ALL_PROJECTS_TURTLE_WAY_H
