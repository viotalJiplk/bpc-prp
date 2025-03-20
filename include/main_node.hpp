#ifndef MAIN_NODE_HPP
#define MAIN_NODE_HPP

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include "io_node.hpp"
#include "motors.hpp"
#include "line_node.hpp"
#include "encoders.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

namespace nodes
{
    class MainNode : public rclcpp::Node {
    public:
        // Constructor (takes pointers to existing nodes)
        MainNode(
            std::shared_ptr<Motors> motors, 
            std::shared_ptr<LineNode> line, 
            std::shared_ptr<Encoder> encoders,
            std::shared_ptr<IoNode> ionode
        );

        // Destructor
        ~MainNode();

        // Line following control
        // called by button press, will calibrate the line sensors and then follow line
        static void FollowLine();
        
    private:
        // From these variables it is possible to call methods of nodes they contain
        std::shared_ptr<Motors> motors_;
        std::shared_ptr<LineNode> line_;
        std::shared_ptr<Encoder> encoders_;
        std::shared_ptr<IoNode> ionode_;

        // Subscriber for button numbers and its callbask function
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr button_subscriber_;
        void button_callback(std_msgs::msg::UInt8_<std::allocator<void>>::SharedPtr msg);
    };
}



#endif //MAIN_NODE_HPP
