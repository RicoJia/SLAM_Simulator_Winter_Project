// Created by ricojia on 1/28/20.
///\file

/// \brief This node is composed of two parts: a laser scan simulator, and a "real world" environment for robot pose calculation.The reason for that is to reduce the delay caused by
/// publishing and subscribing to the real pose of the robot
/// FakeLaserScan simulates the endpoints of 360 laser beams.
/// RealWorld simulates the real world pose of a diff drive robot, after adding noise to the wheels for slippage simulation. The node also determines the encoder
/// values of wheels, which will have a drift.
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
///   joint_states_pure (/sensor_msgs/JointState): topic to publish joint states on Rviz

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
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

using std::string;
using rigid2d::Transform2D;
using rigid2d::Vector2D;
using rigid2d::Twist2D;
constexpr double PI = 3.14159265;
constexpr char FRAME_ID[] = "base_link";

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


//----------------------------------------------------------------------------------------------


using namespace rigid2d;

///\brief This is the node for simulating the following items in the "real world" with specified Gaussian noise:
/// - right and left wheel velocities of a differential drive robot
/// - transform between the /map and /odom frame
class RealWorld{
public:
    /// \brief Default constructor
    RealWorld();
    /// \brief constructor
    explicit RealWorld(ros::NodeHandle& nh, ros::NodeHandle& nh2);

private:
    double wheel_base, wheel_radius;
    int frequency;
    ros::Subscriber sub;
    ros::Publisher map_odom_pub;
    ros::Publisher joint_state_odom_pub;
    ros::Publisher joint_state_pub;
    tf::TransformBroadcaster map_odom_broadcaster;
    tf::TransformListener listener;
    ros::Time current_time;

    string map_frame_id;
    string odom_frame_id;
    string left_wheel_joint;
    string right_wheel_joint;
    rigid2d::DiffDrive diff_drive;
    double slip_miu, odom_miu, odom_drift;

    /// \brief Callback function for receiving joint_state msg.
    /// \param sensor_msgs::JointState&  joint state messages for left and right wheel velocities
    void sub_callback(const sensor_msgs::JointState& msg);

    /// \brief constructing a tf message for transforming robot's pose in /map to /odom frame, based on pose and body twist
    /// \param sensor_msgs::JointState&  joint state messages for left and right wheel velocities
    /// \param tf message (geometry_msgs::TransformStamped)
    geometry_msgs::TransformStamped construct_tf(const rigid2d::Twist2D&);

    ///\brief constructing an odom message to represent odometry of the robot in /map frame
    ///\param pose(Twist2D): robot pose
    ///\param velocity twist (Twist2D): Body twist of the robot after the last update.
    ///\return odom message for pose of the robot(nav_msgs::Odometry)
    nav_msgs::Odometry construct_map_odom_msg(const rigid2d::Twist2D& pose, const rigid2d::Twist2D& velocity_twist);

    ///\brief constructing a joint state message to represent wheel positions and wheel velocities
    ///\param wheel_pos(rigid2d::WheelPos) left and right wheel positions
    ///\param wheel_vel(rigid2d::WheelVel) left and right wheel velocities
    ///\return joint state message for left and right wheels (sensor_msgs::JointState)
    sensor_msgs::JointState construct_joint_state_msg(const rigid2d::WheelPos& wheel_pos, const rigid2d::WheelVel& wheel_vel);
};

WheelPos get_new_wheel_pos(WheelPos original_wheel_pos, const WheelVel& noisy_wheel_vel){
    original_wheel_pos.theta_l += noisy_wheel_vel.u_l;
    original_wheel_pos.theta_r += noisy_wheel_vel.u_r ;
    return original_wheel_pos;
}

WheelVel add_noise(double u_l, double u_r, double mean, double miu){
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(mean,miu);
    double _u_l = u_l+ distribution(generator);
    double _u_r = u_r + distribution(generator);
    return  WheelVel(_u_l, _u_r);
}

RealWorld::RealWorld(){}
RealWorld::RealWorld(ros::NodeHandle& nh, ros::NodeHandle& nh2):diff_drive()
{
    nh.getParam("/RealWorld/wheel_base", wheel_base);
    nh.getParam("/RealWorld/wheel_radius", wheel_radius);
    nh.getParam("/RealWorld/frequency", frequency);

    nh2.getParam("odom_frame_id", odom_frame_id);
    nh2.getParam("right_wheel_joint", right_wheel_joint);
    nh2.getParam("left_wheel_joint",left_wheel_joint);
    nh2.getParam("map_frame_id",map_frame_id);

    nh2.getParam("slip_miu",slip_miu);
    nh2.getParam("odom_miu",odom_miu);
    nh2.getParam("odom_drift",odom_drift);

    map_odom_pub = nh.advertise<nav_msgs::Odometry>("map_odom", 50);
    joint_state_odom_pub = nh.advertise<sensor_msgs::JointState>("joint_states_odom", 50);
    joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 50);
    current_time = ros::Time::now();
    sub = nh.subscribe("/joint_states_pure", 10, &RealWorld::sub_callback, this);
    auto init_pose = Twist2D();  //default pose
    diff_drive = DiffDrive(init_pose, wheel_base, wheel_radius);
}



nav_msgs::Odometry RealWorld::construct_map_odom_msg(const rigid2d::Twist2D& pose_twist,const rigid2d::Twist2D& velocity_twist ){
    nav_msgs::Odometry odom_msg;
    auto current_time = ros::Time::now();
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = map_frame_id;
    odom_msg.child_frame_id = odom_frame_id;

    odom_msg.pose.pose.position.x = pose_twist.x;
    odom_msg.pose.pose.position.y = pose_twist.y;
    odom_msg.pose.pose.position.z = 0.0;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pose_twist.theta);
    odom_msg.pose.pose.orientation = odom_quat;

    odom_msg.twist.twist.linear.x = velocity_twist.x;
    odom_msg.twist.twist.linear.y = velocity_twist.y;
    odom_msg.twist.twist.angular.z = velocity_twist.theta;
    return odom_msg;
}


sensor_msgs::JointState RealWorld::construct_joint_state_msg(const rigid2d::WheelPos& wheel_pos, const rigid2d::WheelVel& wheel_vel){
    sensor_msgs::JointState joint_state_msg;
    joint_state_msg.header.stamp = ros::Time::now();
    joint_state_msg.name = {left_wheel_joint, right_wheel_joint};
    joint_state_msg.position = {wheel_pos.theta_l, wheel_pos.theta_r};
    joint_state_msg.velocity = {wheel_vel.u_l, wheel_vel.u_r};
    return joint_state_msg;
}


geometry_msgs::TransformStamped RealWorld::construct_tf(const rigid2d::Twist2D& pose_twist){
    geometry_msgs::TransformStamped odom_trans;
    tf::StampedTransform transform;
    current_time = ros::Time::now();
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = map_frame_id;
    odom_trans.child_frame_id = odom_frame_id;

    //calculate map to odom. if nothing is available, publish [0,0,0]
    try{
        listener.lookupTransform("base_link", "odom", ros::Time(0), transform);     //TODO: or reverse?

        tf::Quaternion q = transform.getRotation();
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        double x_bo,y_bo;
        x_bo = transform.getOrigin().x();
        y_bo = transform.getOrigin().y();

        Transform2D Tsb(Vector2D( pose_twist.x, pose_twist.y), pose_twist.theta);
        Transform2D Tbo(Vector2D(x_bo, y_bo),yaw);
        auto Tso = Tsb*Tbo;
        Twist2D Xso = Tso.displacement();

        odom_trans.transform.translation.x = Xso.x;
        odom_trans.transform.translation.y = Xso.y;
        odom_trans.transform.translation.z = 0.0;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(Xso.theta);
        odom_trans.transform.rotation = odom_quat;
    }
    catch (tf::TransformException &ex){
        odom_trans.transform.translation.x = 0.0;
        odom_trans.transform.translation.y = 0.0;
        odom_trans.transform.translation.z = 0.0;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0.0);
        odom_trans.transform.rotation = odom_quat;
    }

    return odom_trans;
}


void RealWorld::sub_callback(const sensor_msgs::JointState& msg){

    //broadcast transform here
    auto pose_twist = diff_drive.get_pose();
    geometry_msgs::TransformStamped map_odom_trans = construct_tf(pose_twist);
    map_odom_broadcaster.sendTransform(map_odom_trans);
    // update odom here

    auto left_iterator = std::find(msg.name.begin(),msg.name.end(), left_wheel_joint);
    int left_index = std::distance(msg.name.begin(), left_iterator);
    auto right_iterator = std::find(msg.name.begin(),msg.name.end(), right_wheel_joint);
    int right_index = std::distance(msg.name.begin(), right_iterator);

    // Adding noise to wheel veloctity, for the real world itself and for the odometer.
    auto wheel_vel_map = add_noise(msg.velocity[left_index], msg.velocity[right_index], 0, slip_miu);
    auto wheel_vel_odometer = add_noise(wheel_vel_map.u_l, wheel_vel_map.u_r, odom_drift, odom_miu);

    //Update wheel positions, based on the noisy wheel velocities
    auto wheel_pos = diff_drive.wheelPositions();        //actual wheel_positions, after the noisy increment
    wheel_pos = get_new_wheel_pos(wheel_pos, wheel_vel_map);    //this is not updating the diff_drive parameter. This is for update odometry only.

    diff_drive.updateOdometry(wheel_pos.theta_l, wheel_pos.theta_r);    //update the real world twist of the robot.
    diff_drive.update_wheel_pos(wheel_vel_map);

    auto velocity_twist_map = diff_drive.wheelsToTwist(wheel_vel_map);

    //construct odom msg and publish here, also, don't forget to add some noise in.
    nav_msgs::Odometry map_odom_msg = construct_map_odom_msg(pose_twist,velocity_twist_map);        //TODO: Cause problem with old pose_twist?
    map_odom_pub.publish(map_odom_msg);

    // global map odom
    GLOBAL_MAP_ODOM_MSG = map_odom_msg;

    //construct joint state msgs for simulator and odometer
    sensor_msgs::JointState joint_state_msg = construct_joint_state_msg(wheel_pos,wheel_vel_map);
    sensor_msgs::JointState joint_state_odom_msg = construct_joint_state_msg(wheel_pos, wheel_vel_odometer);
    joint_state_pub.publish(joint_state_msg);
    joint_state_odom_pub.publish(joint_state_odom_msg);
}


int main(int argc, char**argv){
    ros::init(argc, argv, "RealWorld");
    ros::NodeHandle nh;
    ros::NodeHandle nh2("~");
    RealWorld real_world(nh,nh2);
    FakeLaserScan laserScan(nh, nh2);
    ros::Rate r(laserScan.get_pub_frequency());
    while(nh.ok()) {
        laserScan.pub_scan_msgs();
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
