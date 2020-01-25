/// \fil
/// \brief This file will serve as a node named "turtle_rect", which will send velocity command to turtlesim that makes the turtle travel in a rectangular shape.
///
/// PARAMETERS:
///    x - The x coordinate of the lower left corner of a rectangle
///    y - The y coordinate of the lower left corner of a rectangle
///    width - The width of the rectangle
///    height - The height of the rectangle
///    trans_vel - The translational velocity of the robot
///    rot_vel - The rotational velocity of the robot
///    frequency - The frequency of the control loop
/// PUBLISHES:
///     /pose_error (tsim/PoseError): The error between pose estimation from dead-reckoning and the real pose.
///     /turtle1/cmd_vel (geometry_msgs/Twist): velocity command to turtlesim
/// SUBSCRIBES:
///    /turtle1/pose (turtlesim/Pose)
/// SERVICES:
///     traj_reset (tsim/traj_set): Reset the turtle's position to the lower left corner of the rectangle.
///     However, this service does not restart the trajectory, i.e, the turtle's will finish the rest of the old trajectory

#include "tsim/turtle_rect.hpp"

bool reset_now = false;
double x_dead_reckoning = 0;
double y_dead_reckoning = 0;
double theta_dead_reckoning = 0;
int turn_counter = 0;
double x_pos =  0;
double y_pos = 0;
tsim::PoseError pose_error_msg;


bool pub_pose_error = false;

/// \brief Set pen status of the turtle trajectory. If status is true, no trajectory. If status is false, there is trajectory in pink
void drop_pen(ros::NodeHandle nh, bool status)
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

/// \brief Teleport the turtle to the lower left corner of the box. It will wait until the real position is less than dist_thre away from the desired position
void teleport_2_bottom(ros::NodeHandle nh, double x, double y)
{
    ros::service::waitForService("/turtle1/teleport_absolute", 100);
    ros::ServiceClient teleport_client = nh.serviceClient<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute");
    turtlesim::TeleportAbsolute teleportabsolute_srv;
    teleportabsolute_srv.request.x = x;        //can we use a constructor here? Better way of doing this?
    teleportabsolute_srv.request.y = y;
    teleportabsolute_srv.request.theta = 0;

   double dist_thre = 1;
   while ( pow((x_pos - x),2)+ pow((y_pos - y),2) > dist_thre){
   //wait to get to the destination correctly
   teleport_client.call(teleportabsolute_srv);
   ros::spinOnce();
   }
}

/// \brief Call back function for traj_reset service.
bool traj_reset_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){
    reset_now = true;
    return true;
}

/// \brief Call back function for subscribed topic "/turtle1/pose"
void pose_callback(const turtlesim::Pose& pose_msg)
{
    double theta = pose_msg.theta;
    double x = x_pos = pose_msg.x;
    double y = y_pos = pose_msg.y;
    pose_error_msg.x_error = x-x_dead_reckoning;
    pose_error_msg.y_error = y-y_dead_reckoning;
    pose_error_msg.theta_error = theta-theta_dead_reckoning;
    if (pose_error_msg.theta_error > 3.1415 && pose_error_msg.x_error< 6.2831){
        pose_error_msg.theta_error -= 6.2831;
    }
    if (pose_error_msg.theta_error < -3.1415 && pose_error_msg.x_error > -6.2831){
        pose_error_msg.theta_error += 6.2831;
    }
    pub_pose_error = true;
}

/// \brief Linear pose estimation for Dead-reckoning
void update_linear_pos(double v, double dt){
    if (turn_counter==4) turn_counter = 0;
    switch (turn_counter){
        case 0: x_dead_reckoning += v*dt; //go left
            break;
        case 1: y_dead_reckoning += v*dt;   //go up
            break;
        case 2: x_dead_reckoning -= v*dt;   //go right
            break;
        case 3: y_dead_reckoning -= v*dt;
            break;
    }
}

/// \brief Angular pose estimation for Dead-reckoning
void update_angular_pos(double w, double dt){
    theta_dead_reckoning += w*dt;
    if (theta_dead_reckoning > 6.28){
        theta_dead_reckoning -= 6.28;
    }
}

int main(int argc, char**argv)
{
    double x, y, width, height, trans_vel, rot_vel;
    int frequency;
    ros::init(argc, argv,"turtle_rect");
    ros::NodeHandle nh;
    //setup service
    ros::ServiceServer service = nh.advertiseService("traj_reset", traj_reset_callback);
    //setup subscriber
    ros::Subscriber sub = nh.subscribe("/turtle1/pose", 1, pose_callback);

    if (nh.getParam("/turtle_rect/x", x)){      //How to get rid of turtle_rect??
        ROS_INFO("x: %f", x);
    }

    if (nh.getParam("/turtle_rect/y", y)){
        ROS_INFO("y: %f", y);
    }
    if (nh.getParam("/turtle_rect/width", width)){
        ROS_INFO("width: %f", width);
    }
    if (nh.getParam("/turtle_rect/height", height)){
        ROS_INFO("height: %f", height);
    }
    if (nh.getParam("/turtle_rect/trans_vel", trans_vel)){
        ROS_INFO("trans_vel: %f",trans_vel);
    }
    if (nh.getParam("/turtle_rect/rot_vel", rot_vel)){
        ROS_INFO("rot_vel: %f",rot_vel);
    }
    if (nh.getParam("/turtle_rect/frequency", frequency)){
        ROS_INFO("frequency: %d",frequency);
    }

    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
    ros::Publisher pose_error_pub = nh.advertise<tsim::PoseError>("pose_error", 1000);
    ros::Rate loop_rate(frequency);
    State turtle_state = State::go_forward;
    ros::Time init_time = ros::Time::now();
    double turn_time = 3.1415927/2.0/rot_vel;

    drop_pen(nh, true);
    teleport_2_bottom(nh,x, y);
    drop_pen(nh, false);

    x_dead_reckoning = x;
    y_dead_reckoning = y;

    while (ros::ok){

        if (reset_now == true){
            teleport_2_bottom(nh, x, y);
            reset_now = false;
        }

        if (pub_pose_error){
            pose_error_pub.publish(pose_error_msg);
            pub_pose_error = false;
        }

        geometry_msgs::Twist msg;
        if (turtle_state == State ::go_forward){
            msg.linear.x = trans_vel;
            msg.angular.z = 0;
            if (ros::Time::now()-init_time>=ros::Duration(1)){
                turtle_state = State ::turn;
                init_time = ros::Time::now();
                turn_counter += 1;
            }
            update_linear_pos(trans_vel, 1.0/frequency);
        }
        else{
            msg.linear.x = 0;
            msg.angular.z = rot_vel;
            if (ros::Time::now() - init_time>= ros::Duration(turn_time)){
                turtle_state = State::go_forward;
                init_time = ros::Time::now();
            }
            update_angular_pos(rot_vel, 1.0/frequency);
        }

        cmd_vel_pub.publish(msg);
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
