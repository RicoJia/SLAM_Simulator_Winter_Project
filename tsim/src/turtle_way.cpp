//
// Created by ricojia on 1/24/20.
//

#include "tsim/turtle_way.hpp"
using std::vector;
using namespace rigid2d;

TurtleWay::TurtleWay(){}

TurtleWay::TurtleWay(ros::NodeHandle& nh, ros::NodeHandle& nh2): wp_vec(), max_vel(Twist2D(0.5, 0.5, 0.0)), wp()
{

    vector<double> waypoints_x, waypoints_y;
    nh.getParam("/TurtleWay/wheel_base", wheel_base);
    nh.getParam("/TurtleWay/wheel_radius", wheel_radius);
    nh.getParam("/TurtleWay/frequency", frequency);
    nh.getParam("/TurtleWay/waypoints_x", waypoints_x);
    nh.getParam("/TurtleWay/waypoints_y", waypoints_y);

    for (unsigned i = 0; i < waypoints_x.size(); ++i){
        Vector2D waypoint(waypoints_x[i], waypoints_y[i]);
        wp_vec.push_back(waypoint);
    }

    double init_heading = 0.0;
    wp = Waypoints(init_heading, wheel_base, wheel_radius, wp_vec, max_vel, frequency);

    pose_sub = nh.subscribe("/turtle1/pose", 1, &TurtleWay::sub_callback, this);
    error_pub = nh.advertise<tsim::PoseError>("pose_error", 50);
    vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 50);

    turn_off_pen(nh, true);
    teleport_2_bottom(nh, init_heading);
    turn_off_pen(nh, false);
}

void TurtleWay::sub_callback(const turtlesim::Pose& pose_msg){

    double theta_err = pose_msg.theta - wp.get_pose().theta;
    double x_err = pose_msg.x - wp.get_pose().x;
    double y_err = pose_msg.y - wp.get_pose().y;

    tsim::PoseError pose_error_msg;
    pose_error_msg.theta_error = theta_err;
    pose_error_msg.x_error = x_err;
    pose_error_msg.y_error = y_err;
    error_pub.publish(pose_error_msg);
}

void TurtleWay::publish_velocity_commands() {
    auto commanded_vel = wp.nextWaypoint();
    geometry_msgs::Twist cmd_vel;
    cmd_vel.angular.z = commanded_vel.theta;
    cmd_vel.linear.x = commanded_vel.x;
    vel_pub.publish(cmd_vel);
}

void TurtleWay::turn_off_pen(ros::NodeHandle& nh, bool status)
{
    ros::service::waitForService("/turtle1/set_pen", 100);
    ros::ServiceClient client = nh.serviceClient<turtlesim::SetPen>("/turtle1/set_pen");
    turtlesim::SetPen setpen_srv;
    setpen_srv.request.r = 255;        //can we use a constructor here?
    setpen_srv.request.g = 192;
    setpen_srv.request.b = 203;
    setpen_srv.request.width = 1;
    setpen_srv.request.off = status;

    client.call(setpen_srv);
}

void TurtleWay::teleport_2_bottom(ros::NodeHandle& nh, double init_heading)
{
    ros::service::waitForService("/turtle1/teleport_absolute", 100);
    ros::ServiceClient teleport_client = nh.serviceClient<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute");
    turtlesim::TeleportAbsolute teleportabsolute_srv;
    teleportabsolute_srv.request.x = wp_vec[0].x;        //can we use a constructor here? Better way of doing this?
    teleportabsolute_srv.request.y = wp_vec[0].y;
    teleportabsolute_srv.request.theta = init_heading;

    teleport_client.call(teleportabsolute_srv);

}

int main(int argc, char** argv){
    ros::init(argc, argv, "TurtleWay");
    ros::NodeHandle nh;
    ros::NodeHandle nh2("~");
    TurtleWay turtleway(nh,nh2);
    ros::Rate rate( turtleway.frequency );
    while (ros::ok()){
        turtleway.publish_velocity_commands();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
