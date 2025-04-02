#pragma once

#include <cstdint>
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

        // Functions to set LED colors
        void set_led_color(uint8_t led_number, uint8_t R, uint8_t G, uint8_t B);
        void set_all_leds_color(uint8_t R, uint8_t G, uint8_t B);

    private:

        // Variable to store the last received button press value
        int button_pressed_ = -1;

        // Variable to store current LEDs state
        uint8_t leds[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0};

        // Subscriber for button press messages
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr button_subscriber_;

        // Publisher for LED RGB values
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr led_publisher_;

        // Publisher for button values
        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr button_publisher_;

        // Callback - preprocess received message
        void on_button_callback(std_msgs::msg::UInt8_<std::allocator<void>>::SharedPtr msg);
    };
}
