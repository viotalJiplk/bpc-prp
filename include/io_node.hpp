#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

namespace nodes {
    class IoNode : public rclcpp::Node {
    public:
        // Constructor
        IoNode();
        // Destructor (default)
        ~IoNode() override = default;

        // Function to retireve the last pressed button value
        int get_button_pressed() const;

    private:
        std::shared_ptr<rclcpp::Subscription<std_msgs::msg::UInt8_<std::allocator<void>>>> subscriber;
        // Variable to store the last received button press value
        int button_pressed_ = -1;

        // Subscriber for button press messages
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr button_subscriber_;
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr led_publisher_;

        // Callback - preprocess received message
        void on_button_callback(std_msgs::msg::UInt8_<std::allocator<void>>::SharedPtr msg);
    };
}
