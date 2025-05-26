// main.cpp
// BPC-PRP project 2025
// xvarec06 & xruzic56
//
// ROS2 executor spinning all our nodes.


#include <rclcpp/rclcpp.hpp>
#include "io_node.hpp"
#include "line_node.hpp"
#include "ultrasound_node.hpp"
#include "ultrasound_sensor_node.hpp"
#include "main_node.hpp"
#include "keyboard_node.hpp"
#include "lidar_sensor_node.hpp"
#include "lidar_node.hpp"
#include "imu_node.hpp"
#include "aruco_node.hpp"
#include "mazeNode.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    // Create an executor (for handling multiple nodes)
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    auto kinematics = std::make_shared<nodes::KinematicsNode>(executor);
    executor->add_node(kinematics);

    auto ioNode = std::make_shared<nodes::IoNode>();
    executor->add_node(ioNode);

    auto line = std::make_shared<nodes::LineNode>(kinematics, ioNode);
    executor->add_node(line);

    auto ultrasound = std::make_shared<nodes::UltrasoundNode>(kinematics, ioNode);
    executor->add_node(ultrasound);

    auto ultrasoundSensor = std::make_shared<nodes::UltrasoundSensorNode>();
    executor->add_node(ultrasoundSensor);

    auto keyboard = std::make_shared<nodes::KeyboardInputNode>();
    executor->add_node(keyboard);

    auto lidar_sensor = std::make_shared<nodes::LidarSensorNode>();
    executor->add_node(lidar_sensor);

    auto lidar = std::make_shared<nodes::LidarNode>(kinematics, ioNode, ultrasound);
    executor->add_node(lidar);

    auto imu = std::make_shared<nodes::ImuNode>(kinematics);
    executor->add_node(imu);

    auto aruco = std::make_shared<nodes::ArucoNode>();
    aruco->init();
    executor->add_node(aruco);

    auto mazeNode = std::make_shared<nodes::MazeNode>(ioNode, kinematics, lidar, imu, ultrasound, line); //TODO add aruco
    executor->add_node(mazeNode);

    auto mainNode = std::make_shared<nodes::MainNode>(ioNode, line, kinematics, ultrasound, keyboard, lidar, imu, mazeNode);
    executor->add_node(mainNode);

    // Run the executor (handles callbacks for both nodes)
    executor->spin();

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
