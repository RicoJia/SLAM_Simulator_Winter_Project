//
// Created by ricojia on 1/20/20.
//

#ifndef INC_495_NAV_ALL_PROJECTS_WAYPOINTS_H
#define INC_495_NAV_ALL_PROJECTS_WAYPOINTS_H

#include <vector>
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"

namespace rigid2d {

    /// \brief theta and distance error threshold for robot's rotation and translation.
    const double WAYPOINTS_THETA_THRESHOLD = 0.01;
    const double WAYPOINTS_DISTANCE_THRESHOLD = 0.01;

    /// \brief A loop waypoint twist generator -  starts at the first waypoint, travel through all the waypoint and come back to the first waypoint.
    /// \param init_heading - robot's initial heading
    /// \param update_frequency - frequency to call nextWaypoint function
    /// \param max_velocities - Twist2d maximum robot body velocity(per second) in body frame.
    /// \param waypoints_vec -  vector of all waypoints. the first waypoint is the current target.
    /// Note: This implementation is NOT most efficient as we used erase on a vector, which might be slower than list.

    class Waypoints {

    public:
        /// \brief Default constructor that sets empty waypoints array and (0,0) as current_position
        Waypoints():
            current_position(0,0,0),
            velocity_lim(0,0,0),
            waypoints_vec()
            {}

        /// \brief Constructor that input waypoint locations. The waypoint array should contain starting point as the first waypoint!!
        /// \param Twist2D body_twist - body frame twist for the next time instant (NOT in 1 second)
        explicit Waypoints(const double init_heading, const double update_frequency, const Twist2D max_velocities, const std::vector<Vector2D> wps):
            current_position(Twist2D(init_heading, wps[0].x, wps[0].y)),
            velocity_lim(max_velocities.theta/update_frequency, max_velocities.x/update_frequency, 0),
            waypoints_vec(wps)
            {
                if (max_velocities.y!=0)
                    std::cerr<<"Warning: body velocity in y direction should not be zero. Now it's been corrected to zero."<<std::endl;
                update_target();
            };

        /// \brief Determine the velocity command for the next time instant
        /// \param Twist2D current_pose: current pose of the robot.
        Twist2D nextWaypoint(const Twist2D& current_pose);

        bool reset_waypoints(const double& init_heading, const std::vector<Vector2D>&);

        Twist2D current_position;
        Twist2D velocity_lim;
        std::vector<Vector2D> waypoints_vec;

        /// \brief get theta error between required angle rotation to the current target and the current heading
        /// \param Twist2D current pose
        double get_theta_error();

        /// \brief Finding the target waypoint for the next time instant.
        void update_target();

        /// \brief Update the current position based on the body twist in the body frame.
        /// X, Y velocities are NOT updated with the new angle. Assume velocity is constant since the last call of this function.
        /// \param Twist2D velocities of the robot - angular and linear
        void update_current_position(const Twist2D& velocities);

        /// \brief get distance error between the current pose and the current target
        double get_distance_error();

    private:



    };

}

#endif //INC_495_NAV_ALL_PROJECTS_WAYPOINTS_H
