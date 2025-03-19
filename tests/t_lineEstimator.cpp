
#include <iostream>
#include <gtest/gtest.h>
#include "line_node.hpp"

TEST(LineEstimator, line_estimator_test_1) {
    float left_value = 0.5;
    float right_value = 4.6;
    auto result = nodes::LineNode::estimate_descrete_line_pose(left_value, right_value);

    EXPECT_EQ(result, DiscreteLinePose::LineOnRight);
}

// ...

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
