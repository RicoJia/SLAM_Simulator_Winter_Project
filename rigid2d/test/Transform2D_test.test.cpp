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
TEST(Transform2D_Test_Suite, displacement_test){      //you can specify name of the test suite and test case here

    rigid2d::Transform2D transform;
    rigid2d::Twist2D zero_twist;
    EXPECT_DOUBLE_EQ(zero_twist.theta, transform.displacement().theta);
    EXPECT_DOUBLE_EQ(zero_twist.x,transform.displacement().x);
    EXPECT_DOUBLE_EQ(zero_twist.y, transform.displacement().y);
}

/// \brief  Google Gtest for: Input Overloading
/// \param v -
/// \return -
TEST(Transform2D_Test_Suite, input_test){
    rigid2d::Transform2D transform;
    stringstream ss;
    ss<<(1.0)<<endl<<2<<endl<<3<<endl;
    ss>>transform;
    EXPECT_NEAR(1.0, transform.displacement().theta, 0.001);
    EXPECT_NEAR(2.0, transform.displacement().x, 0.001);
    EXPECT_NEAR(3.0, transform.displacement().y, 0.001);
}

/// \brief  Google Gtest for: Output operator overloading
/// \param v -
/// \return -
TEST(Transform2D_Test_Suite, output_test){
    rigid2d::Transform2D transform;
    stringstream ss;
    (ss<<transform);
    const std::string actual= ss.str().c_str();
    ss.str("");     //flush ss
    MatrixXd T = MatrixXd::Identity(3, 3);
    ss<<"******************"<<endl<<T<<endl<<"******************"<<endl;
    const std::string expectation = ss.str().c_str();
    EXPECT_EQ(0, actual.compare(expectation));
}

/// \brief  Google Gtest for: apply a transformation to a Vector2D
/// \param v -
/// \return -
TEST(Transform2D_Test_Suite, Transform_Vector2D_test){
    rigid2d::Vector2D v_test(1.0, 2.0);
    rigid2d::Transform2D transform;
    auto v_result = transform(v_test);
    EXPECT_NEAR(v_result.x, v_test.x, 0.001);
    EXPECT_NEAR(v_result.y, v_test.y, 0.001);
}

/// \brief Google Gtest for: apply a transformation to a Twist2D
/// \param v - the twist to transform
/// \return a twist in the new coordinate system
TEST(Transform2D_Test_Suite, Transform_Twist2D_test){
    rigid2d::Twist2D v_test(0.0,1.0, 2.0);
    rigid2d::Transform2D transform;
    auto v_result = transform(v_test);
    EXPECT_NEAR(v_result.theta, v_test.theta, 0.001);
    EXPECT_NEAR(v_result.x, v_test.x, 0.001);
    EXPECT_NEAR(v_result.y, v_test.y, 0.001);
}

/// \brief Google Gtest for: invert the transformation
TEST(Transform2D_Test_Suite, Transform_inv_test){
    rigid2d::Transform2D transform(1.0);
    rigid2d::Transform2D transform_exp(-1.0);
    auto transform_result = transform.inv();
    EXPECT_NEAR(transform_result.displacement().theta, transform_exp.displacement().theta, 0.001);
    EXPECT_NEAR(transform_result.displacement().x, transform_exp.displacement().x, 0.001);
    EXPECT_NEAR(transform_result.displacement().y, transform_exp.displacement().y, 0.001);
}

/// \brief Google Gtest for: compose this transform with another and store the result
//Transform2D & operator*=(const Transform2D & rhs);
TEST(Transform2D_Test_Suite, Transform_self_mulplication_test){
    rigid2d::Transform2D transform1(1.0);
    rigid2d::Transform2D transform2(-1.0);
    rigid2d::Transform2D transform_exp;
    transform1*=transform2;
    EXPECT_NEAR(transform1.displacement().theta, transform_exp.displacement().theta, 0.001);
    EXPECT_NEAR(transform1.displacement().x, transform_exp.displacement().x, 0.001);
    EXPECT_NEAR(transform1.displacement().y, transform_exp.displacement().y, 0.001);
}

/// \brief Gtest for:  multiply two transforms together, returning their composition
TEST(Transform2D_Test_Suite, Transform_mulplication_test){
    rigid2d::Transform2D transform1(1.0);
    rigid2d::Transform2D transform2(-1.0);
    rigid2d::Transform2D transform_exp;
    auto transform_result = transform1*transform2;
    EXPECT_NEAR(transform_result.displacement().theta, transform_exp.displacement().theta, 0.001);
    EXPECT_NEAR(transform_result.displacement().x, transform_exp.displacement().x, 0.001);
    EXPECT_NEAR(transform_result.displacement().y, transform_exp.displacement().y, 0.001);
}

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
}

int main(int argc, char * argv[]){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
