//
// Created by ricojia on 1/20/20.
//

#ifndef INC_495_NAV_ALL_PROJECTS_WAYPOINTS_H
#define INC_495_NAV_ALL_PROJECTS_WAYPOINTS_H

#include <vector>
#include <ros/ros.h>
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"

using namespace rigid2d;

namespace rigid2d {

    /// \brief theta and distance error threshold for robot's rotation and translation.
    const double WAYPOINTS_THETA_THRESHOLD = 0.01;
    const double WAYPOINTS_DISTANCE_THRESHOLD = 0.2;

    /// \file
    /// \brief A loop waypoint twist generator for a diff drive - Simple Strategy: starts at the first waypoint, travel through all the waypoint and come back to the first waypoint.
    /// If you want to modify this for another model, modify the velocity lim part. You probably want to delete wheel_base and wheel_radius.
    /// PARAMETERS:
    ///     init_heading - robot's initial heading
    ///     update_frequency - frequency to call nextWaypoint function
    ///     max_vel - Twist2d maximum robot body velocity(per second) in body frame.
    ///               Precaution: In order to avoid confusion between two possible angle increment values from Encoders, always make sure translation and rotation velocities (world frame) are less
    ///               than: maximum turning velocity (rad/s): (2*pi* wheel_radius)/(wheel_base*frequency). Maximum translation velocity (m/s): 2*pi*wheel_radius/frequency. In this implementation, we are using 0.9 as a premultiplier.
    ///     wps -  vector of all waypoints. the first waypoint is the current target.

    class Waypoints {

    public:
        /// \brief Default constructor that sets empty waypoints array and (0,0) as current_position
        Waypoints(){}

        /// \brief Constructor that input waypoint locations. The waypoint array should contain starting point as the first waypoint!!
        /// \param Twist2D init_heading: initial heading of the robot.
        /// \param wheel_base: the distance between the center of the two whees
        /// \param wheel_radius: radius of the wheels.
        /// \param wps: waypoint vectors, the first element is the starting position of the robot.

        explicit Waypoints(const double init_heading, double wheel_base, double wheel_radius, const std::vector<Vector2D> wps, Twist2D max_vel,
                           unsigned int frequency):
            dd(init_heading, wps[0].x, wps[0].y, 0.0, 0.0),
            velocity_lim(PI*0.9*wheel_radius/wheel_base*frequency, PI*0.9*wheel_radius*frequency, 0),
            waypoints_vec(wps),
            frequency(frequency)
            {
                update_target();
                if (max_vel.theta <= velocity_lim.theta)
                    velocity_lim.theta = max_vel.theta;
                else
                    ROS_WARN("Warning: User defined maximum angular velocity exceeds encoder's maximum speed threshold. Now the maximum angular velocity is set according the encoder's maximum speed threshold ");

                if (max_vel.x <= velocity_lim.x)
                    velocity_lim.x = max_vel.x;
                else
                    ROS_WARN("Warning: User defined maximum forward velocity exceeds encoder's maximum speed threshold. Now the maximum forward velocity is set according to the encoder's maximum speed threshold ");
            };

        /// \brief Determine the velocity command for the next time instant
        /// \return Twist2D current_pose: current pose of the robot.
        Twist2D nextWaypoint();

        /// \brief Return the current pose of the robot
        ///\return Twist2D pose
        Twist2D get_pose();

        /// \brief Reset waypoints generator to a new heading a set of waypoints
        /// \param init_heading (double): the initial heading of the robot, relative to the positive x axis;
        /// \param waypoints (std::vector<Vector2D>): vector of waypoints, the first element is the coordinate of the starting point.
        void reset_waypoints(const double&, const std::vector<Vector2D>&);


    private:
        DiffDrive dd;
        Twist2D velocity_lim;
        std::vector<Vector2D> waypoints_vec;

        unsigned int frequency;

        /// \brief get theta error between the current pose and the current target
        /// \return theta error (double)
        double get_theta_error();

        /// \brief Finding the target waypoint for the next time instant.
        void update_target();

        /// \brief Update the current position based on the wheel velocity commanded by the nextWaypoint function.
        /// X, Y velocities are NOT updated with the new angle. Assume velocity is constant since the last call of this function.
        /// \param Twist2D velocities of the robot - angular and linear
        void update_current_position(const Twist2D& velocities);

        /// \brief get distance error between the current pose and the current target
        /// \return distance error (double)
        double get_distance_error();
    };

}

#endif //INC_495_NAV_ALL_PROJECTS_WAYPOINTS_H
