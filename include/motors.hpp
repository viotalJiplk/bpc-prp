#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

namespace nodes {
    class Motors : public rclcpp::Node {
    public:
        // Constructor
        Motors();
        // Destructor (default)
        ~Motors() override = default;

        void setMotorsSpeed(uint8_t left, uint8_t right);

    private:
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr motors_publisher_;
    };
}
