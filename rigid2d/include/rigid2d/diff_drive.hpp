//
// Created by ricojia on 1/19/20.
//

#ifndef INC_495_NAV_ALL_PROJECTS_DIFF_DRIVE_H
#define INC_495_NAV_ALL_PROJECTS_DIFF_DRIVE_H

#include "rigid2d/rigid2d.hpp"

/// \file
/// \brief A DiffDrive Simulator. This file is part of the rigid2d namespace - definition of DiffDrive.
/// the robot will turn toward the next waypoint until heading is within a threshold, travel to waypoint until you reach it, then target the next waypoint

namespace rigid2d {

    struct WheelVel{
        double u_l, u_r;
        WheelVel(double u_l = 0, double u_r = 0):u_l(u_l), u_r(u_r){}
    };

    struct WheelPos{
        double theta_l, theta_r;
        WheelPos(double l = 0, double r = 0): theta_l(l), theta_r(r){}
    };

    class DiffDrive
    {
    public:
        /// \brief the default constructor creates a robot at (0,0,0), with a fixed wheel base and wheel radius
        DiffDrive():pose_transform(),
                    wheel_velocities(0.0, 0.0),
                    wheel_positions(0.0, 0.0),
                    wheel_base(0),
                    wheel_radius(0)
                    {}

        /// \brief create a DiffDrive model by specifying the pose, and geometry
        /// \param pose - the current position of the robot
        /// \param wheel_base - the distance between the wheel centers
        /// \param wheel_radius - the raidus of the wheels
        explicit DiffDrive(Twist2D pose, double wheel_base, double wheel_radius):
                                                    pose_transform(Vector2D(pose.x, pose.y), pose.theta),
                                                    wheel_positions(0.0, 0.0),
                                                    wheel_base(wheel_base),
                                                    wheel_radius(wheel_radius)
        {}

        /// \brief create a DiffDrive model by specifying each individual param, and geometry
        /// \param pose - the current position of the robot
        /// \param wheel_base - the distance between the wheel centers
        /// \param wheel_radius - the raidus of the wheels                                                                {}
        explicit DiffDrive(double theta, double x, double y, double l, double r):
                                                                        pose_transform(Vector2D(x, y), theta),
                                                                        wheel_velocities(0.0, 0.0),
                                                                        wheel_positions(0.0, 0.0),
                                                                        wheel_base(l),
                                                                        wheel_radius(r)
        {}
        /// \brief determine the wheel velocities required to make the robot
        ///        move with the desired linear and angular velocities
        /// \param twist - the desired twist in the body frame of the robot
        /// \returns - the wheel velocities to use
        /// \throws std::exception when body twist in y direction is non zero
        WheelVel twistToWheels(const Twist2D&);
//
        /// \brief determine the body twist of the robot from its wheel velocities
        /// \param vel - the velocities of the wheels, assumed to be held constant for one time unit
        /// \returns twist in the original body frame of the
        Twist2D wheelsToTwist(const WheelVel&);
//
        /// \brief Update the robot's odometry based on the current encoder readings and  UPDATE ON ROBOT's current pose.
        /// This function is used on odometer, which reflects the real life odometry data.
        /// Precaution: In order to avoid confusion between two possible angle increment values, always make sure your maximum twist increments for translation and rotation (world frame)are less
        /// than: maximum turning increment: (2*pi* wheel_radius)/wheel_base. Maximum translation increment: 2*pi*wheel_radius
        /// \param left - the left encoder angle (in radians). should take in the total left & right encoder value, not the increment!!
        /// \param right - the right encoder angle (in radians)
        /// \return the velocities of each wheel, assuming that they have been
        /// constant since the last call to updateOdometry  (NOT considering delta_t)
        WheelVel updateOdometry(const double&, const double&);
//
        /// \brief update the odometry of the diff drive robot, assuming that it follows the given body twist for one time  unit , including: pose. This function should be used on the robot control
        /// \param cmd - the body twist command to send to the robot
        void feedforward(const Twist2D&);

        /// \brief get the current pose of the robot, in the world frame.
        /// \return Twist2D
        Twist2D get_pose();

        /// \brief update the current position of the wheels with wheel angle increments
        /// \return WheelPos
        WheelPos update_wheel_pos(const WheelVel&);

        /// \brief get the wheel speeds, based on the last encoder update
        /// \return wheel velocities
        WheelVel wheelVelocities() const;
//
        /// \brief reset the robot to the given position/orientation
        void reset(Twist2D ps);

    private:
        Transform2D pose_transform;
        WheelVel wheel_velocities;
        WheelPos wheel_positions;
        double wheel_base;
        double wheel_radius;
    };
}

#endif //INC_495_NAV_ALL_PROJECTS_DIFF_DRIVE_H
