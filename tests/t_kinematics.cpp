// t_kinematics.cpp
// BPC-PRP project 2025
// xvarec06 & xruzic56
//
// Tests for kinematics algorithm (using GoogleTests).


#include <gtest/gtest.h>
#include "../include/kinematics.hpp"
#include <cmath>

#include "encoders.hpp"

using namespace algorithms;

constexpr double ERROR = 0.001;
constexpr double WHEEL_BASE = 0.128;
constexpr double WHEEL_RADIUS = 0.033;
constexpr double WHEEL_CIRCUMFERENCE = 2 * M_PI * WHEEL_RADIUS;
constexpr int32_t PULSES_PER_ROTATION = 576;



TEST(KinematicsTest, BackwardZeroVelocitySI) {
    const double linear = 0;
    const double angular = 0;
    const double expected_l = 0;
    const double expected_r = 0;

    Kinematics kin(WHEEL_RADIUS, WHEEL_BASE, PULSES_PER_ROTATION);
    auto result = kin.inverse(RobotSpeed {linear, angular});
    EXPECT_NEAR(result.l, expected_l, ERROR);
    EXPECT_NEAR(result.r, expected_r, ERROR);
}

TEST(KinematicsTest, BackwardPositiveLinearVelocitySI) {
    const double linear = 1.0;
    const double angular = 0;
    const double expected_l = 1.0 / WHEEL_CIRCUMFERENCE * 2 * M_PI;
    const double expected_r = 1.0 / WHEEL_CIRCUMFERENCE * 2 * M_PI;

    Kinematics kin(WHEEL_RADIUS, WHEEL_BASE, PULSES_PER_ROTATION);
    auto result = kin.inverse(RobotSpeed {linear,angular});
    EXPECT_NEAR(result.l, expected_l, ERROR);
    EXPECT_NEAR(result.r, expected_r, ERROR);
}

TEST(KinematicsTest, BackwardPositiveAngularVelocitySI) {
    const double linear = 0.0;
    const double angular = 1;
    const double expected_l = -(0.5 * WHEEL_BASE) / WHEEL_CIRCUMFERENCE * (2 * M_PI);
    const double expected_r = +(0.5 * WHEEL_BASE) / WHEEL_CIRCUMFERENCE * (2 * M_PI);

    Kinematics kin(WHEEL_RADIUS, WHEEL_BASE, PULSES_PER_ROTATION);
    auto result = kin.inverse(RobotSpeed{linear, angular});
    EXPECT_NEAR(result.l, expected_l, ERROR);
    EXPECT_NEAR(result.r, expected_r, ERROR);
}

TEST(KinematicsTest, ForwardZeroWheelSpeedSI) {
    const double wheel_l = 0;
    const double wheel_r = 0;
    const double expected_l = 0;
    const double expected_a= 0;

    Kinematics kin(WHEEL_RADIUS, WHEEL_BASE, PULSES_PER_ROTATION);
    auto result = kin.forward(WheelAngularSpeed {wheel_l,wheel_r});
    EXPECT_NEAR(result.v, expected_l, ERROR);
    EXPECT_NEAR(result.w, expected_a, ERROR);
}

TEST(KinematicsTest, ForwardEqualWheelSpeedsSI) {
    const double wheel_l = 1;
    const double wheel_r = 1;
    const double expected_l = WHEEL_RADIUS;
    const double expected_a= 0;

    Kinematics kin(WHEEL_RADIUS, WHEEL_BASE, PULSES_PER_ROTATION);
    auto result = kin.forward(WheelAngularSpeed {wheel_l,wheel_r});
    EXPECT_NEAR(result.v, expected_l, ERROR);
    EXPECT_NEAR(result.w, expected_a, ERROR);
}

TEST(KinematicsTest, ForwardOppositeWheelSpeedsSI) {
    const double wheel_l = -1;
    const double wheel_r = 1;
    const double expected_l = 0;
    const double expected_a= (WHEEL_RADIUS / (0.5 * WHEEL_BASE));

    Kinematics kin(WHEEL_RADIUS, WHEEL_BASE, PULSES_PER_ROTATION);
    auto result = kin.forward(WheelAngularSpeed {wheel_l,wheel_r});
    EXPECT_NEAR(result.v, expected_l, ERROR);
    EXPECT_NEAR(result.w, expected_a, ERROR);
}

TEST(KinematicsTest, ForwardAndBackwardSI) {
    const double wheel_l = 1;
    const double wheel_r = -0.5;

    Kinematics kin(WHEEL_RADIUS, WHEEL_BASE, PULSES_PER_ROTATION);
    auto lin_ang = kin.forward(WheelAngularSpeed {wheel_l,wheel_r});
    auto result = kin.inverse(lin_ang);
    EXPECT_NEAR(result.l, wheel_l, ERROR);
    EXPECT_NEAR(result.r, wheel_r, ERROR);
}


TEST(KinematicsTest, ForwardAndBackwardEncoderDiff) {
    const int encoder_l = 0;
    const int encoder_r = 576;

    Kinematics kin(WHEEL_RADIUS, WHEEL_BASE, PULSES_PER_ROTATION);
    auto d_robot_pose = kin.forward(Encoders {encoder_l,encoder_r});
    auto result = kin.inverse(d_robot_pose);
    EXPECT_NEAR(result.l, encoder_l, 1);
    EXPECT_NEAR(result.r, encoder_r, 1);
}

// Main function to run all tests
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
