#include <rclcpp/rclcpp.hpp>
#include "RosExampleClass.hpp"
#include "io_node.hpp"
#include "encoders.hpp"
#include "motors.hpp"

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

    auto encoder = std::make_shared<nodes::Encoder>();
    executor->add_node(encoder);

    auto motor = std::make_shared<nodes::Motors>();
    executor->add_node(motor);

    auto ioNode = std::make_shared<nodes::IoNode>(motor);
    executor->add_node(ioNode);

    // Run the executor (handles callbacks for both nodes)
    executor->spin();

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
