// encoders.hpp
// BPC-PRP project 2025
// xvarec06 & xruzic56
//
// Header file for encoders data processing node (used by kinematics).


#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include "generalNode.hpp"

namespace nodes {
    class Encoder : public GeneralNode {
    public:
        // Constructor
        Encoder();
        // Destructor (default)
        ~Encoder() override = default;
        uint32_t getRightEncoderState();
        uint32_t getLeftEncoderState();
    private:
        uint32_t count;
        std::atomic<uint32_t> left;
        std::atomic<uint32_t> right;
        std::shared_ptr<rclcpp::Subscription<std_msgs::msg::UInt32MultiArray_<std::allocator<void>>>> UInt8MultiArray_;

        // Subscriber for button press messages
        rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr encoderSubscriber_;
        rclcpp::Publisher<std_msgs::msg::UInt32MultiArray>::SharedPtr encoderPublisher_;

        // Callback - preprocess received message
        void on_encoder_callback(std_msgs::msg::UInt32MultiArray_<std::allocator<void>>::SharedPtr msg);
    };
}
