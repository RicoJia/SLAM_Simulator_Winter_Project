//
// Created by ricojia on 1/19/20.
//

#include <gtest/gtest.h>
#include "rigid2d/diff_drive.hpp"
#include <Eigen/Dense>

using Eigen::MatrixXd;
using std::stringstream;
using std::endl;

using namespace rigid2d;

//TEST(DiffDrive_Test_Suite, constructor_test){      //you can specify name of the test suite and test case here
//
//    rigid2d::DiffDrive dd;
//    EXPECT_NEAR(dd.get_pose().theta, 0.0, 0.001);
//    EXPECT_NEAR(dd.get_pose().x, 0.0, 0.001);
//    EXPECT_NEAR(dd.get_pose().y, 0.0, 0.001);
//
//    rigid2d::DiffDrive dd2(1.0, 2.0, 3.0,4.0,5.0);
//    EXPECT_NEAR(dd2.get_pose().theta, 1.0, 0.001);
//    EXPECT_NEAR(dd2.get_pose().x, 2.0, 0.001);
//    EXPECT_NEAR(dd2.get_pose().y, 3.0, 0.001);
//}
//
//TEST(DiffDrive_Test_Suite, twistToWheels_test){      //you can specify name of the test suite and test case here
//    rigid2d::DiffDrive dd(0, 0, 0, 1, 1);
//    rigid2d::Twist2D body_twist (0, 1, 0);
//    auto wheelVel = dd.twistToWheels(body_twist);
//
//    EXPECT_NEAR(wheelVel.u_l, 1, 0.001);
//    EXPECT_NEAR(wheelVel.u_r, 1, 0.001);
//}
//
//TEST(DiffDrive_Test_Suite, wheelsToTwist_test){      //you can specify name of the test suite and test case here
//    rigid2d::DiffDrive dd(0, 0, 0, 1, 1);
//
//    rigid2d::WheelVel wheelVel1(1.0, 1.0);
//    rigid2d::WheelVel wheelVel2(1.0, 0.0);
//    rigid2d::WheelVel wheelVel3(2.0, 1.0);
//    rigid2d::WheelVel wheelVel4(1.0, 2.0);
//
//    auto bodyTwist1 = dd.wheelsToTwist( wheelVel1);
//    auto bodyTwist2 = dd.wheelsToTwist( wheelVel2);
//    auto bodyTwist3 = dd.wheelsToTwist( wheelVel3);
//    auto bodyTwist4 = dd.wheelsToTwist( wheelVel4);
//
//    EXPECT_NEAR(bodyTwist1.theta, 0.0, 0.001); EXPECT_NEAR(bodyTwist1.x, 1.0, 0.001); EXPECT_NEAR(bodyTwist1.y, 0.0, 0.001);
//    EXPECT_NEAR(bodyTwist2.theta, -1.0, 0.001); EXPECT_NEAR(bodyTwist2.x, 0.5, 0.001); EXPECT_NEAR(bodyTwist2.y, 0.0, 0.001);
//    EXPECT_NEAR(bodyTwist3.theta, -1.0, 0.001); EXPECT_NEAR(bodyTwist3.x, 1.5, 0.001); EXPECT_NEAR(bodyTwist3.y, 0.0, 0.001);
//    EXPECT_NEAR(bodyTwist4.theta, 1.0, 0.001); EXPECT_NEAR(bodyTwist4.x, 1.5, 0.001); EXPECT_NEAR(bodyTwist4.y, 0.0, 0.001);
//}
//
//TEST(DiffDrive_Test_Suite, reset_test){
//    rigid2d::DiffDrive dd(0, 0, 0, 1, 1);
//    EXPECT_NEAR(dd.get_pose().theta, 0.0, 0.001);EXPECT_NEAR(dd.get_pose().x, 0.0, 0.001);EXPECT_NEAR(dd.get_pose().y, 0.0, 0.001);
//    rigid2d::Twist2D ps(1,2,3);
//    dd.reset(ps);
//    EXPECT_NEAR(dd.get_pose().theta, 1.0, 0.001);EXPECT_NEAR(dd.get_pose().x, 2.0, 0.001);EXPECT_NEAR(dd.get_pose().y, 3.0, 0.001);
//}
//
TEST(DiffDrive_Test_Suite, Odometry_Integration_test){      //including updateOdometry and feedforward functions
    rigid2d::DiffDrive dd(0, 0, 0, 1, 1);
    rigid2d::Twist2D ps(0.0,0.0,0.0);

//Pure Translation
    auto wheelVel1 = dd.updateOdometry(1.0, 1.0);
    EXPECT_NEAR(wheelVel1.u_l, 1.0, 0.001); EXPECT_NEAR(wheelVel1.u_r, 1.0, 0.001);
    auto bodyTwist1 = dd.wheelsToTwist( wheelVel1);
    EXPECT_NEAR(bodyTwist1.theta, 0.0, 0.001); EXPECT_NEAR(bodyTwist1.x, 1.0, 0.001); EXPECT_NEAR(bodyTwist1.y, 0.0, 0.001);
    dd.reset(ps);
    dd.feedforward(bodyTwist1);
    EXPECT_NEAR(dd.get_pose().theta, 0.0, 0.001);EXPECT_NEAR(dd.get_pose().x, 1.0, 0.001);EXPECT_NEAR(dd.get_pose().y, 0.0, 0.001);

    dd.reset(ps);
    //Pure Rotation
    wheelVel1 = dd.updateOdometry(-1.0, 1.0);
    EXPECT_NEAR(wheelVel1.u_l, -1.0, 0.001); EXPECT_NEAR(wheelVel1.u_r, 1.0, 0.001);
    bodyTwist1 = dd.wheelsToTwist(wheelVel1);
    EXPECT_NEAR(bodyTwist1.theta, 2.0, 0.001); EXPECT_NEAR(bodyTwist1.x, 0.0, 0.001); EXPECT_NEAR(bodyTwist1.y, 0.0, 0.001);
    dd.reset(ps);
    dd.feedforward(bodyTwist1);
    EXPECT_NEAR(dd.get_pose().theta, 2.0, 0.001);EXPECT_NEAR(dd.get_pose().x, 0.0, 0.001);EXPECT_NEAR(dd.get_pose().y, 0.0, 0.001);

    //Rotation + Translation
    dd.reset(ps);
    auto wheelVel3 = dd.updateOdometry(rigid2d::PI/2.0, rigid2d::PI*1.0);
    auto bodyTwist3 = dd.wheelsToTwist( wheelVel3);
    EXPECT_NEAR(bodyTwist3.theta, rigid2d::PI/2.0, 0.001);
    EXPECT_NEAR(bodyTwist3.x, rigid2d::PI*3.0/4.0, 0.001);
    EXPECT_NEAR(bodyTwist3.y, 0.0, 0.001);
    dd.reset(ps);
    dd.feedforward(bodyTwist3);
    EXPECT_NEAR(dd.get_pose().theta, rigid2d::PI/2.0, 0.001);
    EXPECT_NEAR(dd.get_pose().x, 1.5, 0.001);
    EXPECT_NEAR(dd.get_pose().y, 1.5, 0.001);
    }

int main(int argc, char * argv[]){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
