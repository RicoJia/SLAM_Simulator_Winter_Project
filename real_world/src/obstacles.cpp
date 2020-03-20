//
// Created by ricojia on 1/27/20.
//

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <vector>

#include <geometry_msgs/Quaternion.h>

//#include <tf/transform_datatypes.h>

constexpr char frame_id[] = "map";

visualization_msgs::Marker make_circular_marker(double x, double y, double circular_obstacle_radius){
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time();
    marker.ns = "Obstacles";

    static unsigned int marker_id = 0;
    marker.id = marker_id;
    ++marker_id;

    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.5;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = circular_obstacle_radius*2;
    marker.scale.y = circular_obstacle_radius*2;
    marker.scale.z = 1.0;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    return marker;
}

visualization_msgs::Marker make_rectangular_marker(double x, double y, double length, double width, double theta){
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "Walls";

    static unsigned int marker_id = 0;
    marker.id = marker_id;
    ++marker_id;

    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.25;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);


    marker.pose.orientation = odom_quat;
    marker.scale.x = length;
    marker.scale.y = width;
    marker.scale.z = 0.5;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.5;
    return marker;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "Obstacles");
    ros::NodeHandle n;
    ros::NodeHandle nh2("~");

    ros::Publisher vis_pub = n.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 0 );
    visualization_msgs::MarkerArray marker_array;

    std::vector<double> circular_obstacles_x;
    std::vector<double> circular_obstacles_y;
    double radius;
    nh2.getParam("circular_obstacles_x", circular_obstacles_x);
    nh2.getParam("circular_obstacles_y", circular_obstacles_y);
    nh2.getParam("circular_obstacle_radius", radius);

    std::vector<double> rectangular_obstacles_x;
    std::vector<double> rectangular_obstacles_y;
    std::vector<double> rectangular_obstacles_length;
    std::vector<double> rectangular_obstacles_width;
    std::vector<double> rectangular_obstacles_orientation;
    nh2.getParam("rectangular_obstacles_x", rectangular_obstacles_x);
    nh2.getParam("rectangular_obstacles_y", rectangular_obstacles_y);
    nh2.getParam("rectangular_obstacles_length", rectangular_obstacles_length);
    nh2.getParam("rectangular_obstacles_width", rectangular_obstacles_width);
    nh2.getParam("rectangular_obstacles_orientation", rectangular_obstacles_orientation);
    //rectangular_obstacles_orientation: [0.0]

    for (unsigned int i = 0; i<circular_obstacles_x.size(); ++i){
        visualization_msgs::Marker marker = make_circular_marker(circular_obstacles_x.at(i), circular_obstacles_y.at(i), radius);
        marker_array.markers.push_back(marker);
    }
    
    for (unsigned int i = 0; i<rectangular_obstacles_x.size(); ++i){
        visualization_msgs::Marker marker = make_rectangular_marker(rectangular_obstacles_x.at(i), rectangular_obstacles_y.at(i), rectangular_obstacles_length.at(i), rectangular_obstacles_width.at(i), rectangular_obstacles_orientation.at(i));
        marker_array.markers.push_back(marker);
    }
    
    //only if using a MESH_RESOURCE marker type:

    ros::Rate r(1.0);
    while(n.ok()) {
        vis_pub.publish(marker_array);
        r.sleep();
    }

}

