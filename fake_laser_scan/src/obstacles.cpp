//
// Created by ricojia on 1/27/20.
//

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <vector>

constexpr char frame_id[] = "map";

visualization_msgs::Marker make_marker(double x, double y, double obstacle_radius){
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
    marker.scale.x = obstacle_radius;
    marker.scale.y = obstacle_radius;
    marker.scale.z = 1.0;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    return marker;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "Obstacles");
    ros::NodeHandle n;
    ros::NodeHandle nh2("~");

    ros::Publisher vis_pub = n.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 0 );
    visualization_msgs::MarkerArray marker_array;

    std::vector<double> obstacles_x;
    std::vector<double> obstacles_y;
    double radius;
    nh2.getParam("obstacles_x", obstacles_x);
    nh2.getParam("obstacles_y", obstacles_y);
    nh2.getParam("obstacle_radius", radius);
    //
//    //test
//    std::cout<<"obstacles: "<<obstacles_x[1]<<std::endl;

    for (unsigned int i = 0; i<obstacles_x.size(); ++i){
        visualization_msgs::Marker marker = make_marker(obstacles_x.at(i), obstacles_y.at(i), radius);
        marker_array.markers.push_back(marker);
    }
    //only if using a MESH_RESOURCE marker type:

    ros::Rate r(10.0);
    while(n.ok()) {
        vis_pub.publish(marker_array);
        r.sleep();
    }

}

