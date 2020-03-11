////
//// Created by ricojia on 3/4/20.
////
//
//
//#include "real_world/LandmarkList.h"
//#include "sensor_msgs/JointState.h"
//#include <ros/ros.h>
//#include <string>
//#include <vector>
//#include <tf/transform_broadcaster.h>
//
//using std::string;
//
//class EkfNode{
//public:
//    EkfNode(){}
//    EkfNode(ros::NodeHandle& nh, ros::NodeHandle& nh2);
//private:
//    string odom_frame_id;
//    string map_frame_id;
//    tf::TransformBroadcaster br;
//    std::vector <double> joint_positions;
//    std::vector <double> last_joint_positions;
//    self.landmarks =
//};
//
//EkfNode::EkfNode(ros::NodeHandle& nh, ros::NodeHandle& nh2){
//
//}
//
//
//int main(int argc, char**argv){
//    ros::init(argc, argv, "SLAM_testbench");
//    ros::NodeHandle nh;
//    ros::NodeHandle nh2("~");
//    EkfNode ekf_node(nh,nh2);
//    ros::spin();
//    return 0;
//}
