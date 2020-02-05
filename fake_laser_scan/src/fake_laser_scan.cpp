/// \file
/// \brief  Publishes laser scan messages on base_link frame.

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

using rigid2d::Transform2D;
using rigid2d::Vector2D;
using rigid2d::Twist2D;

constexpr double PI = 3.14159265;
constexpr char FRAME_ID[] = "base_link";

//Helper functions:

///\brief Coordinate system for rectangular obstacles
///\param Frames (Transform2D) Inverse transformation of the frame on four vertices of a rectangle in the world frame.
/// The Frames are set up following these rules: 1. right hand frames 2. positive axes of any frame points towards another two sides.
class RectTracker{
public:
    RectTracker(){}

    explicit RectTracker(double x, double y, double theta, double length, double width){
        Transform2D T_center(Vector2D(x,y), theta);
        double d_half = length/2.0; double w_half = width/2.0;
        vertex_frames_inv[0] = (T_center*Transform2D(Vector2D(-1*d_half, -1*w_half), 0.0)).inv();
        vertex_frames_inv[1]= (T_center*Transform2D(Vector2D( 1*d_half, -1*w_half), PI/2.0)).inv();
        vertex_frames_inv[2] = (T_center*Transform2D(Vector2D( 1*d_half,  1*w_half), PI)).inv();
        vertex_frames_inv[3]= (T_center*Transform2D(Vector2D(-1*d_half,  1*w_half), 3.0*PI/2.0)).inv();
    }
    ///\brief Checks if a laser beam will hit part of the rectangle. If so, it will return the scan distance. Otherwise, it will return the scan_max
    ///\param scanner_trans (Transform2D): world frame pose of the scanner
    ///\param endpoint_trans (Transform2D): world frame pose of the laserbeam
    ///\param scan_max (double): maximum scan range
    /// \return scan distance
    double get_rect_scan_distance(const Transform2D& scanner_trans, const Transform2D& endpoint_trans, const double& scan_max);
private:
    Transform2D vertex_frames_inv[4];
};


double RectTracker::get_rect_scan_distance(const Transform2D& scanner_trans, const Transform2D& endpoint_trans, const double& scan_max){
    for (auto vertex_frame: vertex_frames_inv){
        auto end_point = vertex_frame * endpoint_trans;
        if (end_point.displacement().y <= 0) return scan_max;
    }

    for (auto vertex_frame: vertex_frames_inv){
        auto scanner = vertex_frame*scanner_trans;
        auto scanner_position = scanner.displacement();
        if (scanner_position.y <= 0){
            auto end_point_position = ( vertex_frame * endpoint_trans).displacement();
            double ret_scan_distance = scan_max/( (end_point_position.y - scanner_position.y)/abs(scanner_position.y));
            return ret_scan_distance;
        }
    }
}

class FakeLaserScan{
public:
    FakeLaserScan(){}
    explicit FakeLaserScan(ros::NodeHandle n, ros::NodeHandle nh2):num_readings(360)
    {

        scan_pub = n.advertise<sensor_msgs::LaserScan>("fakeScan", 50);
        map_sub = n.subscribe("map_odom", 10, &FakeLaserScan::map_sub_callback, this);
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
    ros::Subscriber map_sub;
    Transform2D T_bs;
    std::vector<RectTracker> rect_tracker_vec;

    ///\brief update the range and bearing of a landmark relative to the laser scanner. The bearing is [0, 2*pi)
    void update_range_and_heading (double& range, double& bearing, unsigned int i);


    ///\brief update pose transformation T_body_to_s, where s is the fixed frame, body is the body frame.
    void map_sub_callback(const nav_msgs::Odometry& odom_msg);

    /// \brief populate the scan vector for each angle with val
    void populate_scan_range(sensor_msgs::LaserScan& scan, double val);

    ///\brief update the angle of occupancy (largest angle of view of the obstacle in laser scan), based on the range of the landmark relative to the obstacle
    /// the angle_of_occupancy will be updated to [0, pi]
    double update_angle_of_occupancy(double angle_of_occupancy, double range);

    ///\brief, given the range of obstacle relative to the laser scanner, and the angle of the laser scan, relative to the line connecting the center of a cylindrical obstacle and the laser scanner, we can compute the distance between
    /// the outerior of the circular obstacle and the center of laser scanner.
    /// \return scan distance at total_angle_increment
    double get_scan_distance(double total_angle_increment, double range);

    ///\brief Calculate the index of a scan angle in the scanner_array, given the angle and angle_increment. Scanner_array[0] should be 0 rad, and its last element should be 2*PI.
    ///\return index of the scan angle.
    inline int angle_to_index(double angle, double angle_increment);

    ///\brief Calculate the endpoint of a laser scanner beam in world frame, represented in Transform2D
    ///\param theta: angle between the laser beam and the robot's orientation.
    ///\return endpoint_trans (Transform2D): Transform from world frame to endpoint
    Transform2D get_endpoint_transform(double theta);
};

void FakeLaserScan::pub_scan_msgs(){
    ros::Time scan_time = ros::Time::now();
            //populate the LaserScan message
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

    //TODO: rectangle
    populate_scan_range(scan, scan_radius);

    for(unsigned int i = 0; i < num_readings; ++i){
        for (auto& rect_tracker: rect_tracker_vec){
            double theta = 2*PI/num_readings*i;
            auto T_sp = get_endpoint_transform(theta);  //end point pose
            double scan_distance_rect = rect_tracker.get_rect_scan_distance(T_bs.inv(), T_sp, scan_radius);
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
                angle_of_occupancy = update_angle_of_occupancy(angle_of_occupancy, range);
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
//    for(unsigned int i = 0; i < num_readings; ++i){
//        scan.intensities[i] = 300; //Here intensity is not a very important factor??? Ask Matt.
//        double theta = 2*PI/num_readings*i;
////        update_pose();
//        // update ith angle from the scan on circular obstacles
//        for (unsigned int i = 0; i<circular_obstacles_x.size();++i){
//            double range, bearing;
//            update_range_and_heading(range, bearing, i);
//            if (range < scan_radius+circular_obstacle_radius){
//                if (range<circular_obstacle_radius){
//                    populate_scan_range(scan, 0.0);
//                }
//                else{
//                    double angle_of_occupancy;
//                    double total_angle_increment = 0;
//                    while (angle_of_occupancy > total_angle_increment){
//                        double scan_distance = get_scan_distance(total_angle_increment, range);
//                        scan.ranges[angle_to_index(angles::normalize_angle_positive(bearing + total_angle_increment), scan.angle_increment) ] = scan_distance;
//                        scan.ranges[angle_to_index(angles::normalize_angle_positive(bearing - total_angle_increment), scan.angle_increment) ] = scan_distance;
//                        total_angle_increment+=scan.angle_increment;
////                        total_angle_increment = angle_of_occupancy;
//                    }
//
//                }
//            }
//        }
//
//        //TODO: rectangle
//    }


    scan_pub.publish(scan);
}

int FakeLaserScan::get_pub_frequency(){
    return laser_frequency;
}

void FakeLaserScan::map_sub_callback(const nav_msgs::Odometry& odom_msg){

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

/// \brief get range and heading of each landmark to the robot
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

double FakeLaserScan::update_angle_of_occupancy(double angle_of_occupancy, double range){
    angle_of_occupancy = angles::normalize_angle_positive( asin(circular_obstacle_radius/range) );
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


int main(int argc, char** argv){
    ros::init(argc, argv, "FakeLaserScanner");
    ros::NodeHandle n;
    ros::NodeHandle nh2("~");
    FakeLaserScan laserScan(n, nh2);

    ros::Rate r(laserScan.get_pub_frequency());
    while(n.ok()){
        laserScan.pub_scan_msgs();
        ros::spinOnce();
        r.sleep();
    }
}
