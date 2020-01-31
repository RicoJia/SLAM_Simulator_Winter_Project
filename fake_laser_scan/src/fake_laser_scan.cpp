/// \file
/// \brief  Publishes laser scan messages on base_link frame.

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TransformStamped.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>


constexpr double PI = 3.1415;
constexpr char frame_id[] = "base_link";
static nav_msgs::Odometry nav_msg;

/// \brief get range for a specific heading.
double get_range_for_heading (const double& theta, const std::vector<double>& obstacles_x, const std::vector<double>& obstacles_y ){
    //TODO
    return 0;
}

void sub_callback(const nav_msgs::Odometry& msg){
    //TODO
    nav_msg = msg;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "FakeLaserScanner");

    ros::NodeHandle n;
    ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("fakeScan", 50);
    tf::TransformBroadcaster frame_broadcaster;
    ros::Subscriber nav_sub = n.subscribe("map_odom", 10, sub_callback);

    unsigned int num_readings = 360;
    double laser_frequency = 40;
    double ranges[num_readings];
    double intensities[num_readings];

    int count = 0;
    ros::Rate r(10.0);

//    loading parameters
    ros::NodeHandle nh2("~");
    std::vector<double> obstacles_x;
    std::vector<double> obstacles_y;
    double radius;
    nh2.getParam("obstacles_x", obstacles_x);
    nh2.getParam("obstacles_y", obstacles_y);
    nh2.getParam("obstacle_radius", radius);

    while(n.ok()){
        //generate some fake data for our laser scan
        for(unsigned int i = 0; i < num_readings; ++i){
            ranges[i] = 5;
            intensities[i] = 100 + count;
        }
        ros::Time scan_time = ros::Time::now();
        //populate the LaserScan message
        sensor_msgs::LaserScan scan;
        scan.header.stamp = scan_time;
        scan.header.frame_id = frame_id;
        scan.angle_min = -1*PI;
        scan.angle_max = PI;
        scan.angle_increment = 2*PI / num_readings;
        scan.time_increment = (1 / laser_frequency) / (num_readings);
        scan.range_min = 0.0;
        scan.range_max = 10.0;

        scan.scan_time = (1 / laser_frequency);

        scan.ranges.resize(num_readings);
        scan.intensities.resize(num_readings);
        for(unsigned int i = 0; i < num_readings; ++i){
            scan.intensities[i] = intensities[i];

            double theta = 2*PI/num_readings*i;

            //TODO
            scan.ranges[i] = ranges[i];
        }

        scan_pub.publish(scan);
        ++count;
        r.sleep();
    }
}
