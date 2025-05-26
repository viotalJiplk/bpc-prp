// mazeNode.hpp
// BPC-PRP project 2025
// xvarec06 & xruzic56
//
// Header file for maze escape mission node.

#ifndef MAZENODE_HPP
#define MAZENODE_HPP

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include "io_node.hpp"
#include "kinematics_node.hpp"
#include "lidar_node.hpp"
#include "imu_node.hpp"
#include "ultrasound_node.hpp"
#include "line_node.hpp"
#include <queue>

enum class ArucoTurn{
    Left,
    Right,
    Forward,
    None

};

namespace nodes
{
    class MazeNode : public rclcpp::Node {
    public:
        // Constructor (takes pointers to existing nodes)
        MazeNode(
            std::shared_ptr<IoNode> ionode,
            std::shared_ptr<KinematicsNode> kinematics,
            std::shared_ptr<LidarNode> lidar_node,
            std::shared_ptr<ImuNode> imu_node,
            std::shared_ptr<LineNode> line_node
        );

        // Destructor
        ~MazeNode();
        void start();
        void stop();

    private:
        ArucoTurn arucoExit;
        ArucoTurn arucoTreasure;
        std::mutex arucoMutex;
        // From these variables it is possible to call methods of nodes they contain
        std::function<void()> fsmCallback;
        std::shared_ptr<IoNode> ionode_;
        std::shared_ptr<KinematicsNode> kinematics_;
        std::shared_ptr<LidarNode> lidar_node_;
        std::shared_ptr<ImuNode> imu_node_;
        std::shared_ptr<LineNode> line_node_;

        // Subscriber for button numbers and its callbask function
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr aruco_subscriber_;
        void aruco_callback_(std_msgs::msg::UInt8_<std::allocator<void>>::SharedPtr msg);
    };
}

#endif //MAZENODE_HPP
