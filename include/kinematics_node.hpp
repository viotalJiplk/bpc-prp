// kinematics_node.hpp
// BPC-PRP project 2025
// xvarec06 & xruzic56
//
// Header file for differential drive kinematics node.


#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include "generalNode.hpp"
#include "motors.hpp"
#include "encoders.hpp"
#include "kinematics.hpp"
#include <stack>

struct Encoders{
    uint32_t l; //left
    uint32_t r; //right
};

struct Plan {
    std::function<void(bool)> callback;
    int16_t lMotor;
    int16_t rMotor;
    bool hasFinished;
    bool isInfinite;
    Encoders start;
    algorithms::EncodersChange change;
};

struct planPaused
{
    struct Plan plan;
    uint32_t lEncoder;
    uint32_t rEncoder;
};

namespace nodes {
    class KinematicsNode : public GeneralNode {
    public:
        // Constructor
        KinematicsNode(std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor);
        // Destructor (default)
        ~KinematicsNode() override = default;
        void forward(uint32_t length, int16_t speed, std::function<void(bool)> callback);
        void backward(uint32_t length, int16_t speed, std::function<void(bool)> callback);
        void angle(double angle, int16_t speed, std::function<void(bool)> callback);
        void motorSpeed(int16_t speedL, int16_t speedR, std::function<void(bool)> callback);
        void motorSpeed(int16_t speedL, int16_t speedR, bool isInfinite, std::function<void(bool)> callback);
        void turnLeft(int16_t speed, std::function<void(bool)> callback);
        void turnRight(int16_t speed, std::function<void(bool)> callback);
        void turnBack(int16_t speed, std::function<void(bool)> callback);
        void interruptOp();
        void continueOp();
        void stop();
        void turnSlightlyLeft(int16_t speed, std::function<void(bool)> callback);
        void turnSlightlyRight(int16_t speed, std::function<void(bool)> callback);
    private:
        std::stack<struct planPaused> planStack;
        Plan plan_;
        algorithms::Kinematics* algo_;
        std::atomic<uint32_t> lEncoder;
        std::atomic<uint32_t> rEncoder;
        std::shared_ptr<nodes::Motors> motors_;
        std::shared_ptr<nodes::Encoder> encoders_;
        rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr encodersSubscriber_;
        void on_encoder_callback(std_msgs::msg::UInt32MultiArray_<std::allocator<void>>::SharedPtr msg);
    };
}
