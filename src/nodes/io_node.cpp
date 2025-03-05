#include "io_node.hpp"
#include "helper.hpp"

namespace nodes {
    IoNode::IoNode(): Node("buttonsReader") {
            // Initialize the subscriber
             button_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
                Topic::buttons, 1, std::bind(&IoNode::on_button_callback, this, std::placeholders::_1));
            led_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(Topic::set_rgb_leds, 1);
    }

    int IoNode::get_button_pressed() const{
    }
    void IoNode::on_button_callback(std_msgs::msg::UInt8_<std::allocator<void>>::SharedPtr msg) {
        std::cout << msg->data << std::endl;
        auto leds = std_msgs::msg::UInt8MultiArray();
        leds.data = {255, 0, 0};
        led_publisher_->publish(leds);
    }
    // ...
}
