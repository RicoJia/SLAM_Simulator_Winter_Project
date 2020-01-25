//
// Created by ricojia on 1/20/20.
//

#include <gtest/gtest.h>
#include "../include/rigid2d/waypoints.hpp"
#include <math.h>
//#include "rigid2d/waypoints.hpp"

using std::endl;
using std::cout;
using std::vector;
using namespace rigid2d;

TEST(Waypoints_Test_Suite, constructor_test){      //you can specify name of the test suite and test case here
    Waypoints wp;

    vector<Vector2D> wp_vec = {Vector2D(1,1),Vector2D(2,2)};
    double update_frequency = 0;
    Twist2D max_velocity (PI, 1, 0);
    Waypoints wp2(PI, update_frequency, max_velocity, wp_vec);
}

//
//TEST(Waypoints_Test_Suite, get_theta_error){      //you can specify name of the test suite and test case here
//
//vector<Vector2D> wp_vec = {Vector2D(0,0),Vector2D(0,4.0)};
//double update_frequency = 1;
//double init_heading = 0;
//Twist2D max_velocity (PI, 1.0, 0.0);
//
//Waypoints wp2(init_heading, update_frequency, max_velocity, wp_vec);
//    EXPECT_NEAR(wp2.get_theta_error(), PI/2.0, 0.001);
//}

//TEST(Waypoints_Test_Suite, update_target_test){      //you can specify name of the test suite and test case here
//    vector<Vector2D> wp_vec = {Vector2D(1,1),Vector2D(2,2)};
//    double update_frequency = 0;
//    Twist2D max_velocity (PI, 1, 0);
//    Waypoints wp2(PI, update_frequency, max_velocity, wp_vec);
//    wp2.update_target();
//    EXPECT_EQ(wp2.waypoints_vec[0].x, Vector2D(1,1).x);
//    EXPECT_EQ(wp2.waypoints_vec[0].y, Vector2D(1,1).y);
//}

//TEST(Waypoints_Test_Suite, update_current_position_test){      //you can specify name of the test suite and test case here
//    Twist2D vel(PI, 1, 2);
//
//    vector<Vector2D> wp_vec = {Vector2D(0,0),Vector2D(2,2)};
//    double update_frequency = 1;
//    Twist2D max_velocity (PI, 1.0, 0.0);
//    Waypoints wp2(PI, update_frequency, max_velocity, wp_vec);
//    wp2.update_current_position(vel);
//
//    EXPECT_EQ(wp2.current_position.theta, 0.0);
//    EXPECT_EQ(wp2.current_position.x, -1.0);
//    EXPECT_NEAR(wp2.current_position.y, 0.0, 0.001);
//}


/// \brief get distance error between the current pose and the current target
//double get_distance_error();
//TEST(Waypoints_Test_Suite, get_distance_error_test){      //you can specify name of the test suite and test case here
//    Twist2D vel(PI, 1, 2);
//
//    vector<Vector2D> wp_vec = {Vector2D(0,0),Vector2D(4,0)};
//    double update_frequency = 1;
//    Twist2D max_velocity (PI, 1.0, 0.0);
//    Waypoints wp2(PI, update_frequency, max_velocity, wp_vec);
//
//    EXPECT_NEAR(wp2.get_distance_error(), 4.0, 0.001);
//}

/// \brief Determine the velocity command for the next time instant
/// \param Twist2D current_pose: current pose of the robot.

//TEST(Waypoints_Test_Suite, nextWaypoint_test){      //This test is invalid!
//
//    //Pure translation
//    vector<Vector2D> wp_vec = {Vector2D(0,0),Vector2D(0.5,0.0)};
////    double update_frequency = 1;
//    double init_heading = 0;
//    double
//    Twist2D max_velocity (PI, 1.0, 0.0);
//    Waypoints wp2(init_heading, update_frequency, max_velocity, wp_vec);
//
//    auto cmd_vel = wp2.nextWaypoint();
//    EXPECT_NEAR(cmd_vel.theta, 0.0, 0.001);
//    EXPECT_NEAR(cmd_vel.x, 0.5, 0.001);
//    EXPECT_NEAR(cmd_vel.y, 0.0, 0.001);
//
//    //Pure Rotation
//    init_heading = PI;
//    wp_vec = {Vector2D(0,0),Vector2D(0.0,4.0)};
//    wp2.reset_waypoints(init_heading, wp_vec);
//    cmd_vel = wp2.nextWaypoint();
//    EXPECT_NEAR(cmd_vel.theta, -PI/2.0, 0.001);
//    EXPECT_NEAR(cmd_vel.x, 0.0, 0.001);
//    EXPECT_NEAR(cmd_vel.y, 0.0, 0.001);
//
//    EXPECT_NEAR(wp2.dd.get_pose().theta, PI/2.0, 0.001);
//    EXPECT_NEAR(wp2.dd.get_pose().x, 0.0, 0.001);
//    EXPECT_NEAR(wp2.dd.get_pose().y, 0.0, 0.001);
//
//    // march on!
//    cmd_vel = wp2.nextWaypoint();
//    EXPECT_NEAR(cmd_vel.theta, 0.0, 0.001);
//    EXPECT_NEAR(cmd_vel.x, 1.0, 0.001);
//    EXPECT_NEAR(cmd_vel.y, 0.0, 0.001);
//
//    EXPECT_NEAR(wp2.dd.get_pose().theta, PI/2.0, 0.001);
//    EXPECT_NEAR(wp2.dd.get_pose().x, 0.0, 0.001);
//    EXPECT_NEAR(wp2.dd.get_pose().y, 1.0, 0.001);
//    }

int main(int argc, char * argv[]){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}