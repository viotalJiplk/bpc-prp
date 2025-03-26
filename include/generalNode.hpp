#ifndef GENERALNODE_HPP
#define GENERALNODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16.hpp>

namespace nodes {
    class GeneralNode : public rclcpp::Node {
     public:
        GeneralNode(std::string nodeName, uint16_t topicId);

        ~GeneralNode();
     protected:
        rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr mainPublisher_;
        std_msgs::msg::UInt16 topicId_;
        void publish();
    };
}


#endif //GENERALNODE_HPP
