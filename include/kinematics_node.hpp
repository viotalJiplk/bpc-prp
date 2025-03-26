#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include "generalNode.hpp"
#include "motors.hpp"
#include "encoders.hpp"
#include "kinematics.hpp"

struct Encoders{
    uint32_t l; //left
    uint32_t r; //right
};

struct Plan {
    std::function<void(bool)> callback;
    int16_t lMotor;
    int16_t rMotor;
    bool hasFinished;
    Encoders start;
    algorithms::EncodersChange change;
};

namespace nodes {
    class KinematicsNode : public GeneralNode {
    public:
        // Constructor
        KinematicsNode(std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor);
        // Destructor (default)
        ~KinematicsNode() override = default;
        void forward(uint32_t length, int16_t speed, std::function<void(bool)> callback);
        void angle(double angle, int16_t speed, std::function<void(bool)> callback);
    private:
        Plan plan_;
        algorithms::Kinematics* algo_;
        std::shared_ptr<nodes::Motors> motors_;
        std::shared_ptr<nodes::Encoder> encoders_;
        rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr encodersSubscriber_;
        void on_encoder_callback(std_msgs::msg::UInt32MultiArray_<std::allocator<void>>::SharedPtr msg);
    };
}
