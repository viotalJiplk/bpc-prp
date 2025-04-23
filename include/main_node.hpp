#ifndef MAIN_NODE_HPP
#define MAIN_NODE_HPP

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include "io_node.hpp"
#include "kinematics_node.hpp"
#include "line_node.hpp"
#include "ultrasound_node.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "keyboard_node.hpp"
#include "std_msgs/msg/char.hpp"
#include "lidar_node.hpp"
#include "imu_node.hpp"
#include "aruco_node.hpp"

namespace nodes
{
    class MainNode : public rclcpp::Node {
    public:
        // Constructor (takes pointers to existing nodes)
        MainNode(
            std::shared_ptr<IoNode> ionode,
            std::shared_ptr<LineNode> line, 
            std::shared_ptr<KinematicsNode> kinematics,
            std::shared_ptr<UltrasoundNode> ultrasound,
            std::shared_ptr<KeyboardInputNode> keyboard_input,
            std::shared_ptr<LidarNode> lidar_node,
            std::shared_ptr<ImuNode> imu_node,
            std::shared_ptr<ArucoNode> aruco_node
        );

        // Destructor
        ~MainNode();

        // Line following control
        // called by button press, will calibrate the line sensors and then follow line
        static void FollowLine();

    private:
        // From these variables it is possible to call methods of nodes they contain
        std::shared_ptr<Motors> motors_;
        std::function<void()> lidarCallback;
        std::shared_ptr<LineNode> line_;
        std::shared_ptr<KinematicsNode> kinematics_;
        std::shared_ptr<UltrasoundNode> ultrasound_;
        std::shared_ptr<KeyboardInputNode> keyboard_input_;
        std::shared_ptr<LidarNode> lidar_node_;
        std::shared_ptr<ImuNode> imu_node_;
        std::shared_ptr<ArucoNode> aruco_node_;

        // Subscriber for button numbers and its callbask function
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr button_subscriber_;
        rclcpp::Subscription<std_msgs::msg::Char>::SharedPtr keyboard_subscriber_;
        void keyboard_callback(std_msgs::msg::Char_<std::allocator<void>>::SharedPtr msg);
        void button_callback(std_msgs::msg::UInt8_<std::allocator<void>>::SharedPtr msg);
    };
}



#endif //MAIN_NODE_HPP
