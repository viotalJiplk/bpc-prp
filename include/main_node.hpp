#ifndef MAIN_NODE_HPP
#define MAIN_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include "motors.hpp"
#include "line_node.hpp"
#include "encoders.hpp"

namespace nodes
{
    class MainNode : public rclcpp::Node {
    public:

        MainNode(std::shared_ptr<Motors> motor, std::shared_ptr<LineNode> line, std::shared_ptr<Encoder> encoders);

        ~MainNode();

    private:
        void on_event(std_msgs::msg::UInt16_<std::allocator<void>>::SharedPtr msg);
        std::shared_ptr<rclcpp::Subscription<std_msgs::msg::UInt16_<std::allocator<void>>>> subscriber;
    };
}



#endif //MAIN_NODE_HPP
