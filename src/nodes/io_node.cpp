#include "io_node.hpp"
#include "helper.hpp"

namespace nodes {
    IoNode::IoNode(std::shared_ptr<Motors> motor, std::shared_ptr<LineNode> line): Node("buttonsReader") {
            // Initialize the subscriber
            line_ = line;
            motors_ = motor;
             button_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
                Topic::buttons, 1, std::bind(&IoNode::on_button_callback, this, std::placeholders::_1));
            led_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(Topic::set_rgb_leds, 1);
    }

    int IoNode::get_button_pressed() const{
    }
    void IoNode::on_button_callback(std_msgs::msg::UInt8_<std::allocator<void>>::SharedPtr msg) {
        std::cout << msg->data << std::endl;
        auto leds = std_msgs::msg::UInt8MultiArray();
        if (msg->data == 0) {
            leds.data = {255, 0, 0};
            line_->calibrationStart();
            motors_->setMotorsSpeed(120, 136);
            // motors_->setMotorsSpeed(140, 0);
        } else if (msg->data == 1) {
            leds.data = {0, 255, 0};
            // motors_->setMotorsSpeed(140, 140);
        } else {
            leds.data = {0, 0, 255};
            // motors_->setMotorsSpeed(0, 140);
        }
        led_publisher_->publish(leds);
    }
    // ...
}
