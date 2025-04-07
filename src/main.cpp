#include <rclcpp/rclcpp.hpp>
#include "io_node.hpp"
#include "line_node.hpp"
#include "ultrasound_node.hpp"
#include "ultrasound_sensor_node.hpp"
#include "main_node.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    // Create an executor (for handling multiple nodes)
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    /*
    // Create multiple nodes
    auto node1 = std::make_shared<rclcpp::Node>("node1");
    auto node2 = std::make_shared<rclcpp::Node>("node2");

    // Create instances of RosExampleClass using the existing nodes
    auto example_class1 = std::make_shared<RosExampleClass>(node1, "topic1", 1.0);
    auto example_class2 = std::make_shared<RosExampleClass>(node2, "topic2", 2.0);

    // Add nodes to the executor
    executor->add_node(node1);
    executor->add_node(node2);
    */

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

    auto mainNode = std::make_shared<nodes::MainNode>(ioNode, line, kinematics, ultrasound);
    executor->add_node(mainNode);

    // Run the executor (handles callbacks for both nodes)
    executor->spin();

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
