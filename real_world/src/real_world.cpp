// Created by ricojia on 1/28/20.
///\file

/// \brief This node is composed of two parts: a laser scan simulator, and a "real world" environment for robot pose calculation.The reason for that is to reduce the delay caused by
/// publishing and subscribing to the real pose of the robot
/// FakeLaserScan simulates the endpoints of 360 laser beams.
/// RealWorld simulates the real world pose of a diff drive robot, after adding noise to the wheels for slippage simulation, which is from Gaussian noise on the commanded twist.
/// The real_world listens to only the velocity info from Fake Encoders, then it keeps track of the robot's wheel position using that information
/// Also, the real world will set the robot's initial position to (0, 0,0)

/// \param
///    dd: DiffDrive object
///    wheel_base - distance between the two wheels
///    wheel_radius - radius of each wheel
///    frequency - The frequency of the control loop
/// \PUBLISHES:  (after receiving subscribed topic updates)
///    map (nav_msgs/Odometry): odometry message containing the real pose info of the robot in the map frame
///    fakeScan(sensor_msgs/LaserScan): laser scan message containing the range information of each laser beam.
///    joint_states (sensor_msgs/JointStates): real world
/// \BROADCASTS:
///    tf/transform_broadcaster: transform for Rviz
/// \SUBSCRIBES:
///   joint_states_pure (/sensor_msgs/JointState): topic to publish joint states on Rviz. Real world  calculates the actual robot pose based on the the wheel velocity.
/// \SERVICES:
///   get_real_pose / real_world/GetRealPose: return the real pose of the robot.

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TransformStamped.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include "rigid2d/rigid2d.hpp"
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <angles/angles.h>
#include <algorithm>
#include <bits/stdc++.h>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>
#include "rigid2d/diff_drive.hpp"
#include <random>
#include <tf2_ros/static_transform_broadcaster.h>
#include "real_world/LandmarkList.h"
#include "real_world/GetRealPose.h"


using std::string;
using rigid2d::Transform2D;
using rigid2d::Vector2D;
using rigid2d::Twist2D;
using rigid2d::distance;
using rigid2d::angle;
constexpr double PI = 3.14159265;
constexpr char FRAME_ID[] = "base_link";


//----------------------------------------------------------------------------------------------Fake Laser Scan

nav_msgs::Odometry GLOBAL_MAP_ODOM_MSG;


///\brief Helper function as a predicate for sorting vertices of a polygon, based on their y coordinates
///\param Two arbitrary vertices (Vector2D)
///\return true if the first vertex has a larger y coordinate than the second
bool compare_y (const Vector2D& a, const Vector2D& b){
    return a.y<b.y;
}

///\brief Helper function as a predicate for partitioning a vector of vertices into two parts: vertices with positive y coordinates and with negative y coordinates
///\param Arbitrary vertex a (Vector2D)
///\return true if the vertex's y coordinate is greater than 0.
bool find_positive_y(const Vector2D& a){
    return a.y > 0;
}

///\brief finding the x intercept of the connecting line between 2 points
///\param two points (Vector2D)
///\return x intercept of the connecting line between 2 points
double find_x_intercept(const Vector2D& pt1, const Vector2D& pt2){
    if (pt1.y == pt2.y){
        return -1;
    }
    else{
        double v = -1.0*pt1.y/(pt2.y - pt1.y);
        double x =  pt1.x + v*(pt2.x - pt1.x);
        return x;
    }
}

///\brief FakeLaser Scan's Helper Coordinate system for rectangular obstacles
///\param Frames (Transform2D) transformation of the frame on four vertices of a rectangle in the world frame.
/// The Frames are set up following these rules: 1. right hand frames 2. positive axes of any frame points towards another two sides.
class RectTracker{
public:
    RectTracker(){}

    explicit RectTracker(double x, double y, double theta, double length, double width){
        Transform2D T_center(Vector2D(x,y), theta);
        double d_half = length/2.0; double w_half = width/2.0;
        vertex_frames[0] = (T_center*Transform2D(Vector2D(-1*d_half, -1*w_half), 0.0));
        vertex_frames[1]= (T_center*Transform2D(Vector2D( 1*d_half, -1*w_half), PI/2.0));
        vertex_frames[2] = (T_center*Transform2D(Vector2D( 1*d_half,  1*w_half), PI));
        vertex_frames[3]= (T_center*Transform2D(Vector2D(-1*d_half,  1*w_half), 3.0*PI/2.0));
    }

///\brief Checks if a laser beam will hit part of the rectangle. If so, it will return the scan distance. Otherwise, it will return the scan_max
///\param scanner_trans (Transform2D): world frame pose of the scanner
///\param endpoint_trans (Transform2D): world frame pose of the laserbeam
///\param scan_max (double): maximum scan range
/// \return scan distance
    double get_rect_scan_distance(const Transform2D& scanner_trans, const double& scan_max);

private:
    Transform2D vertex_frames[4];
};

double RectTracker::get_rect_scan_distance(const Transform2D& scanner_trans, const double& scan_max){
    //get vector of vertices in the beam frame
    std::vector<Vector2D> vertices_vec(4, Vector2D());
    unsigned int i = 0;
    for (auto& vertex_frame:vertex_frames){
        auto x = (scanner_trans*vertex_frame).displacement();
        vertices_vec[i] = Vector2D(x.x, x.y);
        ++i;
    }
    // sort the vector
    std::sort(vertices_vec.begin(), vertices_vec.end(), compare_y);
    if (vertices_vec.front().y * vertices_vec.back().y >= 0){
        return scan_max;
    }
    else{
        //group vertices based on if their y>0
        auto bound = std::stable_partition(vertices_vec.begin(), vertices_vec.end(), find_positive_y);
        std::vector<Vector2D> negative_y_vertices (vertices_vec.begin(),bound);
        std::vector<Vector2D> positive_y_vertices (bound, vertices_vec.end());
        std::vector<double> x_intercepts( (negative_y_vertices.size())*(positive_y_vertices.size()),0);

        // calculate x intercepts of all possible lines that intersects with x axis
        unsigned int x_intercepts_index = 0;
        for (auto& negative_y_vertex:negative_y_vertices){
            for (auto& positive_y_vertex:positive_y_vertices){
                x_intercepts[x_intercepts_index] = find_x_intercept(negative_y_vertex, positive_y_vertex);
                ++x_intercepts_index;
            }
        }

        //find the intercepts that fall between (0, scan_max), then we take the minimum value as the scan range
        auto x_intercept_bound = std::remove_if(x_intercepts.begin(), x_intercepts.end(),
                                                [&scan_max](const double& x_intercept){return (x_intercept >= scan_max)||(x_intercept <= 0); } );

        double scan_range = scan_max;
        if (x_intercept_bound!=x_intercepts.begin()){
            scan_range = *min_element(x_intercepts.begin(), x_intercept_bound);
        }

        return scan_range;
    }
}

/// \brief  Publishes laser scan messages on base_link frame.
class FakeLaserScan{
public:
    FakeLaserScan(){}
    explicit FakeLaserScan(ros::NodeHandle n, ros::NodeHandle nh2):num_readings(360)
    {

        scan_pub = n.advertise<sensor_msgs::LaserScan>("fakeScan", 50);
        //    loading parameters
        nh2.getParam("laser_frequency", laser_frequency);
        nh2.getParam("scan_radius", scan_radius);
        ranges = std::vector<double>(num_readings,scan_radius);

        nh2.getParam("circular_obstacles_x", circular_obstacles_x);
        nh2.getParam("circular_obstacles_y", circular_obstacles_y);
        nh2.getParam("circular_obstacle_radius", circular_obstacle_radius);

        std::vector<double> rectangular_obstacles_x, rectangular_obstacles_y, rectangular_obstacles_length, rectangular_obstacles_width, rectangular_obstacles_orientation;
        nh2.getParam("rectangular_obstacles_x", rectangular_obstacles_x);
        nh2.getParam("rectangular_obstacles_y", rectangular_obstacles_y);
        nh2.getParam("rectangular_obstacles_length", rectangular_obstacles_length);
        nh2.getParam("rectangular_obstacles_width", rectangular_obstacles_width);
        nh2.getParam("rectangular_obstacles_orientation", rectangular_obstacles_orientation);

        for (unsigned int i = 0; i < rectangular_obstacles_x.size(); ++i){
            RectTracker R_temp(rectangular_obstacles_x[i],
                               rectangular_obstacles_y[i],
                               rectangular_obstacles_orientation[i],
                               rectangular_obstacles_length[i],
                               rectangular_obstacles_width[i]);
            rect_tracker_vec.push_back(R_temp);
        }
    }

    void pub_scan_msgs();
    int get_pub_frequency();

private:
    unsigned int num_readings;
    double scan_radius;
    int laser_frequency;
    std::vector<double> ranges;
    double circular_obstacle_radius;
    ros::Publisher scan_pub;
    tf::TransformBroadcaster frame_broadcaster;
    std::vector<double> circular_obstacles_x;
    std::vector<double> circular_obstacles_y;
    Transform2D T_bs;
    std::vector<RectTracker> rect_tracker_vec;


    ///\brief update the range and bearing of a landmark relative to the laser scanner. The bearing is [0, 2*pi)
    ///\param range(double): range of a landmark relative to the laser scanner (to be updated) in meter
    ///\param bearing(double): bearing of a landmark relative to the laser scanner (to be updated) in rad
    ///\param i (unsigned int): index of a circular landmark in circular_obstacles_x;
    void update_range_and_heading (double& range, double& bearing, unsigned int i);


    ///\brief update pose transformation T_body_to_s, where s is the fixed frame, body is the body frame.
    void map_sub_callback();

    /// \brief populate the scan vector for each angle with val
    /// \param scan (sensor_msgs::LaserScan): empty LaserScan message to be published
    /// \param: val(double): value to populate all beams of the laser scanner with
    void populate_scan_range(sensor_msgs::LaserScan& scan, double val);

    ///\brief update the angle of occupancy (largest angle of view of the obstacle in laser scan), based on the range of the landmark relative to the obstacle
    /// the angle_of_occupancy will be updated to [0, pi]
    ///\param range(double) maximum scan range of Lidar
    ///\return angle of occupancy of a rectangular object in Lidar's view of range
    double update_angle_of_occupancy(double range);

    ///\brief, given the range of obstacle relative to the laser scanner, and the angle of the laser scan, relative to the line connecting the center of a cylindrical obstacle and the laser scanner, we can compute the distance between
    /// the outerior of the circular obstacle and the center of laser scanner.
    ///\param total_angle_increment(double): angle between a certain beam and line connecting landmark center and the laser scanner center
    ///\param range(double): range of a landmark relative to the laser scanner center
    ///\return scan distance at total_angle_increment
    double get_scan_distance(double total_angle_increment, double range);

    ///\brief Calculate the index of a scan angle in the scanner_array, given the angle and angle_increment. Scanner_array[0] should be 0 rad, and its last element should be 2*PI.
    /// \param angle(double): angle of a laser beam, relative to the positive x axis of the robot's body frame
    ///\return index of the scan angle.
    inline int angle_to_index(double angle, double angle_increment);

    ///\brief Calculate the endpoint of a laser scanner beam in world frame, represented in Transform2D
    ///\param theta: angle between the laser beam and the robot's orientation.
    ///\return endpoint_trans (Transform2D): Transform from world frame to endpoint
    Transform2D get_endpoint_transform(double theta);
};

void FakeLaserScan::pub_scan_msgs(){

    map_sub_callback();
    ros::Time scan_time = ros::Time::now();
    sensor_msgs::LaserScan scan;
    scan.header.stamp = scan_time;
    scan.header.frame_id = FRAME_ID;
    scan.angle_min = 0.0;
    scan.angle_max = 2*PI;
    scan.angle_increment = 2*PI / num_readings;
    scan.time_increment = (1.0 / laser_frequency) / (num_readings);
    scan.range_min = 0.0;
    scan.range_max = scan_radius+0.001;

    scan.scan_time = (1.0 / laser_frequency);
    scan.ranges.resize(num_readings);
    scan.intensities.resize(num_readings);

    populate_scan_range(scan, scan_radius);

    for(unsigned int i = 0; i < num_readings; ++i) {
        for (auto& rect_tracker: rect_tracker_vec){
            double theta = 2*PI/num_readings*i;
            Transform2D T_beam_b(-1.0*theta);
            double scan_distance_rect = rect_tracker.get_rect_scan_distance(T_beam_b*T_bs, scan_radius);
            if (scan.ranges[i] > scan_distance_rect)
                scan.ranges[i] = scan_distance_rect;
        }
    }

    // update ith angle from the scan on circular obstacles
    for (unsigned int i = 0; i<circular_obstacles_x.size();++i){
        double range, bearing;
        update_range_and_heading(range, bearing, i);
        if (range < scan_radius+circular_obstacle_radius){
            if (range<circular_obstacle_radius){
                populate_scan_range(scan, 0.0);
            }
            else{
                double angle_of_occupancy;
                angle_of_occupancy = update_angle_of_occupancy(range);
                double total_angle_increment = 0;
                while (angle_of_occupancy > total_angle_increment){
                    double scan_distance = get_scan_distance(total_angle_increment, range);
                    if(scan.ranges[angle_to_index(angles::normalize_angle_positive(bearing + total_angle_increment), scan.angle_increment) ] >= scan_distance)
                        scan.ranges[angle_to_index(angles::normalize_angle_positive(bearing + total_angle_increment), scan.angle_increment) ] = scan_distance;
                    if (scan.ranges[angle_to_index(angles::normalize_angle_positive(bearing - total_angle_increment), scan.angle_increment) ] >= scan_distance)
                        scan.ranges[angle_to_index(angles::normalize_angle_positive(bearing - total_angle_increment), scan.angle_increment) ] = scan_distance;
                    total_angle_increment+=scan.angle_increment;
                }
            }
        }
    }

    scan_pub.publish(scan);
}

int FakeLaserScan::get_pub_frequency(){
    return laser_frequency;
}

void FakeLaserScan::map_sub_callback(){
    const nav_msgs::Odometry& odom_msg = GLOBAL_MAP_ODOM_MSG;
    double x_sb = odom_msg.pose.pose.position.x;
    double y_sb = odom_msg.pose.pose.position.y;

    tf::Quaternion q(
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    auto T_sb = Transform2D(Vector2D(x_sb, y_sb), yaw);
    T_bs = T_sb.inv();
}

void  FakeLaserScan::update_range_and_heading (double& range, double& bearing, unsigned int i){
    Vector2D p_s(circular_obstacles_x[i], circular_obstacles_y[i]);
    auto p_b = T_bs(p_s);
    range = sqrt(pow(p_b.x,2)+pow(p_b.y,2));
    bearing = angles::normalize_angle_positive( atan2(p_b.y, p_b.x) );      //output is [0, 2pi)
}

void FakeLaserScan::populate_scan_range(sensor_msgs::LaserScan& scan, double val){
    for(unsigned int i = 0; i < num_readings; ++i) {
        scan.ranges[i] = val;
        scan.intensities[i] = 200;
    }
}

double FakeLaserScan::update_angle_of_occupancy(double range){
    double angle_of_occupancy = angles::normalize_angle_positive( asin(circular_obstacle_radius/range) );
    return angle_of_occupancy;
}


double FakeLaserScan::get_scan_distance(double total_angle_increment, double range){
    double b = -2.0*range*cos(total_angle_increment);
    double c = pow(range,2.0)-pow(circular_obstacle_radius, 2.0);
    double scan_distance = 1/2.0*( -1*b-sqrt(pow(b,2.0) - 4*c) );
    return scan_distance;
}


inline int FakeLaserScan::angle_to_index(double angle, double angle_increment){
    return (int)(angle/angle_increment);
}

Transform2D FakeLaserScan::get_endpoint_transform(double theta){
    Transform2D Tbp(Vector2D(scan_radius*cos(theta), scan_radius*sin(theta)));
    auto Tsp = (T_bs.inv())*Tbp;
    return Tsp;
}

//----------------------------------------------------------------------------------------------REAL WORLD

using namespace rigid2d;

///\brief This is the node for simulating the following items in the "real world" with specified Gaussian noise:
/// - commanded twist and converted to right and left wheel velocities of a differential drive robot
/// - transform between the /map and /odom frame
/// Covariance of control input and observation is assumed to be a diagonal matrix.

/// \PUBLISH:
///     /joint_states   (sensor_msgs/JointStates) - the real world wheel states of the robot
///     /landmark_location?

/// \BROADCAST:
///     world_map  - Static Identity TF transform
///     world_actual_robot (nav_msgs/Odometry) - Transform between world frame and the actual robot frame. Actual Robot frame is the frame which shows the "real world" pose of the robot.





class RealWorld{
public:
/// \brief Default constructor
    RealWorld();
/// \brief constructor
    explicit RealWorld(ros::NodeHandle& nh, ros::NodeHandle& nh2);

    /// \brief sending transform between the world frame and the actual robot pose
    void send_world_actual_robot_tf();

    /// \brief publishing joint state msgs
    void pub_wheel_joint_states();

    /// \brief publishing landmark info
    void pub_landmark_list();

    /// \brief for landmark update, there might be a mismatch between the robot pose in the filter and the pose for landmark update.
    /// by using this function, we can send landmarks based on the previous robot pose, not the current one.
    void update_last_sent_pose_for_landmarks();

private:
    double wheel_base, wheel_radius;
    int frequency;
    ros::Subscriber sub;
    ros::Publisher joint_state_pub;
    ros::Publisher landmark_list_pub;
    tf2_ros::StaticTransformBroadcaster static_world_map_broadcaster;
    tf::TransformBroadcaster world_actual_robot_broadcaster;
    tf::TransformListener listener;
    ros::Time current_time;

    string map_frame_id;
    string odom_frame_id;
    string world_frame_id;
    string actual_robot_frame_id;
    string left_wheel_joint;
    string right_wheel_joint;
    rigid2d::DiffDrive diff_drive;
    rigid2d::Twist2D last_pose_twist;

    std::vector<double> miu_vel, stddev_vel, miu_z, stddev_z;
    std::vector<double> circular_obstacles_x, circular_obstacles_y;
    double circular_obstacle_radius;

    unsigned int landmark_num;
    const double landmark_init_time;

    WheelPos wheel_pos;
    WheelVel wheel_vel;

/// \brief Callback function for receiving joint_state msg.
/// \param sensor_msgs::JointState&  joint state messages for left and right wheel velocities
    void sub_callback(const sensor_msgs::JointState& msg);

/// \brief constructing a tf message for transforming robot's pose in /map to /odom frame, based on pose and body twist
/// \param sensor_msgs::JointState&  joint state messages for left and right wheel velocities
/// \param tf message (geometry_msgs::TransformStamped)
    geometry_msgs::TransformStamped construct_tf(const rigid2d::Twist2D&, const std::string& frame_id, const std::string& child_frame_id );


///\brief constructing a joint state message to represent wheel positions and wheel velocities
///\param wheel_pos(rigid2d::WheelPos) left and right wheel positions
///\param wheel_vel(rigid2d::WheelVel) left and right wheel velocities
///\return joint state message for left and right wheels (sensor_msgs::JointState)
    sensor_msgs::JointState construct_joint_state_msg(const rigid2d::WheelPos& wheel_pos, const rigid2d::WheelVel& wheel_vel);

/// \brief this is the service for returning the actual pose of the robot.
    ros::ServiceServer get_real_pose_srv;
    bool get_real_pose_srv_callback(real_world::GetRealPoseRequest& , real_world::GetRealPoseResponse& res);
};

double add_noise(double var, double mean, double stddev){
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(mean,stddev);
    double _var = var + distribution(generator);
}

RealWorld::RealWorld():landmark_init_time(ros::Time::now().toSec())
{}
RealWorld::RealWorld(ros::NodeHandle& nh, ros::NodeHandle& nh2):diff_drive(), wheel_vel(), wheel_pos(), landmark_init_time(ros::Time::now().toSec())
{
    nh.getParam("/RealWorld/wheel_base", wheel_base);
    nh.getParam("/RealWorld/wheel_radius", wheel_radius);
    nh.getParam("/RealWorld/frequency", frequency);

    nh2.getParam("odom_frame_id", odom_frame_id);
    nh2.getParam("map_frame_id",map_frame_id);
    nh2.getParam("world_frame_id",world_frame_id);
    nh2.getParam("actual_robot_frame_id",actual_robot_frame_id);

    nh2.getParam("right_wheel_joint", right_wheel_joint);
    nh2.getParam("left_wheel_joint",left_wheel_joint);

    nh2.getParam("miu_vel", miu_vel);
    nh2.getParam("stddev_vel", stddev_vel);
    nh2.getParam("miu_z", miu_z);
    nh2.getParam("stddev_z", stddev_z);
    nh2.getParam("circular_obstacles_x", circular_obstacles_x);
    nh2.getParam("circular_obstacles_y", circular_obstacles_y);
    nh2.getParam("circular_obstacle_radius", circular_obstacle_radius);

    // check if miu, stddev arrays are of the same size.
    if ( miu_vel.size()!=stddev_vel.size() ){
        ROS_FATAL("The size of mean and stddevariance arrays of control input do not match!");
    }
    if ( miu_z.size()!=stddev_z.size() ){
        ROS_FATAL("The size of mean and stddevariance arrays of observation do not match!");
    }

    landmark_num = miu_z.size();

    joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 50);
    landmark_list_pub = nh.advertise<real_world::LandmarkList>("landmarks", 50);
    current_time = ros::Time::now();
    sub = nh.subscribe("/joint_states_pure", 10, &RealWorld::sub_callback, this);
    auto init_pose = Twist2D();  //default pose
    diff_drive = DiffDrive(init_pose, wheel_base, wheel_radius);
    last_pose_twist = Twist2D();

    //send static transform from /world to /map
    geometry_msgs::TransformStamped static_tf_msg = construct_tf(Twist2D(0.0, 0.0, 0.0), world_frame_id, map_frame_id);
    static_world_map_broadcaster.sendTransform(static_tf_msg);

    get_real_pose_srv = nh2.advertiseService("get_real_pose", &RealWorld::get_real_pose_srv_callback, this);
}

bool RealWorld::get_real_pose_srv_callback(real_world::GetRealPoseRequest& , real_world::GetRealPoseResponse& res){
    auto pose =  diff_drive.get_pose();
    res.theta =  pose.theta;
    res.x = pose.x;
    res.y = pose.y;
    return true;
    }

void RealWorld::sub_callback(const sensor_msgs::JointState& msg){

    auto left_iterator = std::find(msg.name.begin(),msg.name.end(), left_wheel_joint);
    int left_index = std::distance(msg.name.begin(), left_iterator);
    auto right_iterator = std::find(msg.name.begin(),msg.name.end(), right_wheel_joint);
    int right_index = std::distance(msg.name.begin(), right_iterator);

//     Adding noise to wheel veloctity, for the real world itself and for the odometer.
    auto noiseless_twist = diff_drive.wheelsToTwist(WheelVel(msg.velocity[left_index], msg.velocity[right_index]));
    double noisy_angular_vel = add_noise(noiseless_twist.theta, miu_vel[0], stddev_vel[0]);
    double noisy_linear_vel = add_noise(noiseless_twist.x, miu_vel[1], stddev_vel[1]);
//    auto noisy_twist = Twist2D(noiseless_twist.theta + noisy_angular_vel, noiseless_twist.x + noisy_linear_vel, noiseless_twist.y);
    auto noisy_twist = Twist2D(noisy_angular_vel, noisy_linear_vel, noiseless_twist.y);
    diff_drive.feedforward(noisy_twist);

    wheel_vel = WheelVel(msg.velocity[left_index], msg.velocity[right_index]);
    diff_drive.update_wheel_pos(wheel_vel);
    wheel_pos = diff_drive.wheelPositions();

}

void RealWorld::update_last_sent_pose_for_landmarks(){
    last_pose_twist = diff_drive.get_pose();
}

void RealWorld::send_world_actual_robot_tf() {
    geometry_msgs::TransformStamped world_actual_robot_trans = construct_tf(diff_drive.get_pose(), world_frame_id, actual_robot_frame_id);
    world_actual_robot_broadcaster.sendTransform(world_actual_robot_trans);
}


geometry_msgs::TransformStamped RealWorld::construct_tf(const rigid2d::Twist2D& pose_twist, const std::string& frame_id, const std::string& child_frame_id )
{
    geometry_msgs::TransformStamped trans;
    tf::StampedTransform transform;
    current_time = ros::Time::now();
    trans.header.stamp = current_time;
    trans.header.frame_id = frame_id;
    trans.child_frame_id = child_frame_id;

    trans.transform.translation.x = pose_twist.x;
    trans.transform.translation.y = pose_twist.y;
    trans.transform.translation.z = 0.0;
    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(pose_twist.theta);
    trans.transform.rotation = quat;
    return trans;
}

void RealWorld::pub_wheel_joint_states() {
    sensor_msgs::JointState joint_state_msg = construct_joint_state_msg(wheel_pos,wheel_vel);
    joint_state_pub.publish(joint_state_msg);
    wheel_vel = 0;

}

void RealWorld::pub_landmark_list() {
    real_world::LandmarkList landmark_list;
//    auto current_pose = diff_drive.get_pose();
    //TODO: recover this
    auto current_pose = last_pose_twist;
    for (unsigned int i = 0; i < circular_obstacles_y.size(); ++i){
        real_world::Landmark landmark;

        landmark.last_update = ros::Time::now().toSec() - landmark_init_time;
        landmark.landmark_id = i;
        landmark.position_stddev[0]= stddev_z[0];
        landmark.position_stddev[1]= stddev_z[1];

        double x = circular_obstacles_x[i];
        double y = circular_obstacles_y[i];
        double range = distance(Vector2D(current_pose.x, current_pose.y), Vector2D(x,y));


        double bearing = angle(Vector2D(x - current_pose.x, y - current_pose.y)) - current_pose.theta;
        double noisy_range = add_noise(range, miu_z[0], stddev_z[0]);
        double noisy_bearing = add_noise(bearing, miu_z[1], stddev_z[1]);
        landmark.range = noisy_range;
        landmark.bearing = noisy_bearing;

        landmark_list.landmarks.push_back(landmark);
    }

    landmark_list_pub.publish(landmark_list);

    std::cout<<"real world is based on: "<<current_pose;


}

sensor_msgs::JointState RealWorld::construct_joint_state_msg(const rigid2d::WheelPos& wheel_pos, const rigid2d::WheelVel& wheel_vel){
    sensor_msgs::JointState joint_state_msg;
    joint_state_msg.header.stamp = ros::Time::now();
    joint_state_msg.name = {left_wheel_joint, right_wheel_joint};
    joint_state_msg.position = {wheel_pos.theta_l, wheel_pos.theta_r};
    joint_state_msg.velocity = {wheel_vel.u_l, wheel_vel.u_r};
    return joint_state_msg;
}



int main(int argc, char**argv){
    ros::init(argc, argv, "RealWorld");
    ros::NodeHandle nh;
    ros::NodeHandle nh2("~");
    RealWorld real_world(nh,nh2);
//    FakeLaserScan laserScan(nh, nh2);
//    ros::Rate r(laserScan.get_pub_frequency());
    int frequency;
    nh2.getParam("frequency", frequency);
    ros::Rate r(frequency);
    int count = 1;
    while(nh.ok()) {

        if (count == 20){
            real_world.pub_landmark_list();

            count = 1;
        }
        else{
            ++count;
        }
        real_world.send_world_actual_robot_tf();
        real_world.update_last_sent_pose_for_landmarks();
        real_world.pub_wheel_joint_states();
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
