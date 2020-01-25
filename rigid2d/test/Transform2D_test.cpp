//
// Created by ricojia on 1/18/20.
//

#include <gtest/gtest.h>
#include "rigid2d/rigid2d.hpp"
#include <Eigen/Dense>

using Eigen::MatrixXd;
using std::stringstream;
using std::endl;

/// \brief  Google Gtest for: displacement_test
/// \param v -
/// \return -
//TEST(Transform2D_Test_Suite, displacement_test){      //you can specify name of the test suite and test case here
//
//    rigid2d::Transform2D transform;
//    rigid2d::Twist2D zero_twist;
//    EXPECT_DOUBLE_EQ(zero_twist.theta, transform.displacement().theta);
//    EXPECT_DOUBLE_EQ(zero_twist.x,transform.displacement().x);
//    EXPECT_DOUBLE_EQ(zero_twist.y, transform.displacement().y);
//}
//
///// \brief  Google Gtest for: Input Overloading
///// \param v -
///// \return -
//TEST(Transform2D_Test_Suite, input_test){
//    rigid2d::Transform2D transform;
//    stringstream ss;
//    ss<<(1.0)<<endl<<2<<endl<<3<<endl;
//    ss>>transform;
//    EXPECT_NEAR(1.0, transform.displacement().theta, 0.001);
//    EXPECT_NEAR(2.0, transform.displacement().x, 0.001);
//    EXPECT_NEAR(3.0, transform.displacement().y, 0.001);
//}
//
///// \brief  Google Gtest for: Output operator overloading
///// \param v -
///// \return -
//TEST(Transform2D_Test_Suite, output_test){
//    rigid2d::Transform2D transform;
//    stringstream ss;
//    (ss<<transform);
//    const std::string actual= ss.str().c_str();
//    ss.str("");     //flush ss
//    MatrixXd T = MatrixXd::Identity(3, 3);
//    ss<<"******************"<<endl<<T<<endl<<"******************"<<endl;
//    const std::string expectation = ss.str().c_str();
//    EXPECT_EQ(0, actual.compare(expectation));
//}
//
///// \brief  Google Gtest for: apply a transformation to a Vector2D
///// \param v -
///// \return -
//TEST(Transform2D_Test_Suite, Transform_Vector2D_test){
//    rigid2d::Vector2D v_test(1.0, 2.0);
//    rigid2d::Transform2D transform;
//    auto v_result = transform(v_test);
//    EXPECT_NEAR(v_result.x, v_test.x, 0.001);
//    EXPECT_NEAR(v_result.y, v_test.y, 0.001);
//}
//
///// \brief Google Gtest for: apply a transformation to a Twist2D
///// \param v - the twist to transform
///// \return a twist in the new coordinate system
//TEST(Transform2D_Test_Suite, Transform_Twist2D_test){
//    rigid2d::Twist2D v_test(0.0,1.0, 2.0);
//    rigid2d::Transform2D transform;
//    auto v_result = transform(v_test);
//    EXPECT_NEAR(v_result.theta, v_test.theta, 0.001);
//    EXPECT_NEAR(v_result.x, v_test.x, 0.001);
//    EXPECT_NEAR(v_result.y, v_test.y, 0.001);
//}
//
///// \brief Google Gtest for: invert the transformation
//TEST(Transform2D_Test_Suite, Transform_inv_test){
//    rigid2d::Transform2D transform(1.0);
//    rigid2d::Transform2D transform_exp(-1.0);
//    auto transform_result = transform.inv();
//    EXPECT_NEAR(transform_result.displacement().theta, transform_exp.displacement().theta, 0.001);
//    EXPECT_NEAR(transform_result.displacement().x, transform_exp.displacement().x, 0.001);
//    EXPECT_NEAR(transform_result.displacement().y, transform_exp.displacement().y, 0.001);
//}
//
///// \brief Google Gtest for: compose this transform with another and store the result
////Transform2D & operator*=(const Transform2D & rhs);
//TEST(Transform2D_Test_Suite, Transform_self_mulplication_test){
//    rigid2d::Transform2D transform1(1.0);
//    rigid2d::Transform2D transform2(-1.0);
//    rigid2d::Transform2D transform_exp;
//    transform1*=transform2;
//    EXPECT_NEAR(transform1.displacement().theta, transform_exp.displacement().theta, 0.001);
//    EXPECT_NEAR(transform1.displacement().x, transform_exp.displacement().x, 0.001);
//    EXPECT_NEAR(transform1.displacement().y, transform_exp.displacement().y, 0.001);
//}
//
///// \brief Gtest for:  multiply two transforms together, returning their composition
//TEST(Transform2D_Test_Suite, Transform_mulplication_test){
//    rigid2d::Transform2D transform1(1.0);
//    rigid2d::Transform2D transform2(-1.0);
//    rigid2d::Transform2D transform_exp;
//    auto transform_result = transform1*transform2;
//    EXPECT_NEAR(transform_result.displacement().theta, transform_exp.displacement().theta, 0.001);
//    EXPECT_NEAR(transform_result.displacement().x, transform_exp.displacement().x, 0.001);
//    EXPECT_NEAR(transform_result.displacement().y, transform_exp.displacement().y, 0.001);
//}
//
////-------------------------------------------------------------------------------------------------
///// \brief Gtest for: -= for Vector2D class
//TEST(Vector2D_Test_Suite, self_subtraction_test){
//    rigid2d::Vector2D v1(2.0,1.0);
//    rigid2d::Vector2D v2(2.0,1.0);
//    v1-=v2;
//    EXPECT_NEAR(0.0, v1.x, 0.001);
//    EXPECT_NEAR(0.0, v1.y, 0.001);
//}
//
///// \brief Gtest for: += for Vector2D class
//TEST(Vector2D_Test_Suite, self_addition_test){
//    rigid2d::Vector2D v1(2.0,1.0);
//    rigid2d::Vector2D v2(2.0,1.0);
//    v1+=v2;
//    EXPECT_NEAR(4.0, v1.x, 0.001);
//    EXPECT_NEAR(2.0, v1.y, 0.001);
//}
//
///// \brief Gtest for: *= for Vector2D class when right hand side is a double
//TEST(Vector2D_Test_Suite, self_multiplication_test){
//    rigid2d::Vector2D v1(2.0,1.0);
//    double rhs = 2.0;
//    v1*=rhs;
//    EXPECT_NEAR(4.0, v1.x, 0.001);
//    EXPECT_NEAR(2.0, v1.y, 0.001);
//}
//
///// \brief Gtest for: * for double*Vector2D
//TEST(Vector2D_Test_Suite, multiplication_from_left_test){
//    rigid2d::Vector2D v1(2.0,1.0);
//    double lhs = 2.0;
//    auto v2 = lhs*v1;
//    EXPECT_NEAR(4.0, v2.x, 0.001);
//    EXPECT_NEAR(2.0, v2.y, 0.001);
//}
//
///// \brief Gtest for: * for Vector2D*double
//TEST(Vector2D_Test_Suite, multiplication_from_right_test){
//    rigid2d::Vector2D v1(2.0,1.0);
//    double rhs = 2.0;
//    auto v2 = v1*rhs;
//    EXPECT_NEAR(4.0, v2.x, 0.001);
//    EXPECT_NEAR(2.0, v2.y, 0.001);
//}
//
///// \brief Gtest for: + for Vector2D+ Vector2D
//TEST(Vector2D_Test_Suite, general_addition_test){
//    rigid2d::Vector2D v1(2.0,1.0);
//    rigid2d::Vector2D v2(2.0,1.0);
//    auto v3 = v1+v2;
//    EXPECT_NEAR(4.0, v3.x, 0.001);
//    EXPECT_NEAR(2.0, v3.y, 0.001);
//}
//
///// \brief Gtest for: + for Vector2D-Vector2D
//TEST(Vector2D_Test_Suite, general_subtraction_test){
//    rigid2d::Vector2D v1(2.0,1.0);
//    rigid2d::Vector2D v2(2.0,1.0);
//    auto v3 = v1-v2;
//    EXPECT_NEAR(0.0, v3.x, 0.001);
//    EXPECT_NEAR(0.0, v3.y, 0.001);
//}
//
///// \brief Gtest for: length of a Vector2D
//TEST(Vector2D_Test_Suite, length_test){
//    rigid2d::Vector2D v1(3.0,4.0);
//    auto l = rigid2d::length(v1);
//    EXPECT_NEAR(5.0, l, 0.001);
//}
//
///// \brief Gtest for: distance between 2 Vector2D objects
//TEST(Vector2D_Test_Suite, distance_test){
//    rigid2d::Vector2D v1(0.0,1.0);
//    rigid2d::Vector2D v2(0.0,2.0);
//    auto l = rigid2d::distance(v1, v2);
//    EXPECT_NEAR(1.0, l, 0.001);
//}
//
///// \brief Gtest for: angle between a vector and the positive x axis.
//TEST(Vector2D_Test_Suite, angle_test){
//    rigid2d::Vector2D v1(1.0,1.0);
//    auto l = rigid2d::angle(v1);
//    EXPECT_NEAR(rigid2d::PI/4.0, l, 0.001);
//}
//
///// \brief Gtest for: angle normalization: any angle -> [-pi, pi] equivalent
//TEST(Vector2D_Test_Suite, angle_normalization_test){
//    double rad1 = 0;
//
//    double rad2= rigid2d::PI*(-5.0/4.0);
//    double rad3 = rigid2d::PI*-6.0/4.0;
//    double rad4 = rigid2d::PI*-7.0/4.0;
//    double rad5 = rigid2d::PI*-8.0/4.0;
//
//    double rad6 = rigid2d::PI*5.0/4.0;
//    double rad7 = rigid2d::PI*6.0/4.0;
//    double rad8 = rigid2d::PI*7.0/4.0;
//    double rad9 = rigid2d::PI*8.0/4.0;
//
//    double rad10 = rigid2d::PI*9.0/4.0;
//    double rad11 = rigid2d::PI*-1;
//    double rad12 = rigid2d::PI;
//
//    EXPECT_NEAR(0.0, rad1, 0.001);
//
//    EXPECT_NEAR(rigid2d::PI*3.0/4.0, rigid2d::normalize_angle(rad2), 0.001);
//    EXPECT_NEAR(rigid2d::PI*2.0/4.0, rigid2d::normalize_angle(rad3), 0.001);
//    EXPECT_NEAR(rigid2d::PI*1.0/4.0, rigid2d::normalize_angle(rad4), 0.001);
//    EXPECT_NEAR(rigid2d::PI*0.0/4.0, rigid2d::normalize_angle(rad5), 0.001);
//
//    EXPECT_NEAR(rigid2d::PI*-3.0/4.0, rigid2d::normalize_angle(rad6), 0.001);
//    EXPECT_NEAR(rigid2d::PI*-2.0/4.0, rigid2d::normalize_angle(rad7), 0.001);
//    EXPECT_NEAR(rigid2d::PI*-1.0/4.0, rigid2d::normalize_angle(rad8), 0.001);
//    EXPECT_NEAR(rigid2d::PI*-0.0/4.0, rigid2d::normalize_angle(rad9), 0.001);
//
//    EXPECT_NEAR(rigid2d::PI*1.0/4.0, rigid2d::normalize_angle(rad10), 0.001);
//    EXPECT_NEAR(rigid2d::PI*-4.0/4.0, rigid2d::normalize_angle(rad11), 0.001);
//    EXPECT_NEAR(rigid2d::PI*4.0/4.0, rigid2d::normalize_angle(rad12), 0.001);
//}

//-------------------------------------------------------------------------------------------------
/// \brief Gtest for: compute the transformation corresponding to a rigid body following a constant twist for one time unit
TEST(Individual_Test_Suite, integralTwist_test){
    rigid2d::Twist2D twist_rot(1.0,0.0,0.0);
    rigid2d::Twist2D twist_trans(0.0,1.0,1.0);
    auto transform_rot = rigid2d::integrateTwist(twist_rot);
    auto transform_trans = rigid2d::integrateTwist(twist_trans);

    EXPECT_NEAR(transform_rot.displacement().theta, 1.0, 0.001);
    EXPECT_NEAR(transform_rot.displacement().x, 0.0, 0.001);
    EXPECT_NEAR(transform_rot.displacement().y, 0.0 , 0.001);
    EXPECT_NEAR(transform_trans.displacement().theta, 0.0, 0.001);
    EXPECT_NEAR(transform_trans.displacement().x, 1.0, 0.001);
    EXPECT_NEAR(transform_trans.displacement().y, 1.0 , 0.001);

    rigid2d::Twist2D twist_transrot(-1.0,1.5,0.0);
    auto transform_transrot = rigid2d::integrateTwist(twist_transrot);

}

int main(int argc, char * argv[]){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
