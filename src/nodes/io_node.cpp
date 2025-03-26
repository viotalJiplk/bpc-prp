#include "io_node.hpp"
#include "helper.hpp"

namespace nodes {
    IoNode::IoNode(std::shared_ptr<KinematicsNode> kinematics, std::shared_ptr<LineNode> line): Node("buttonsReader") {
        // Initialize the subscriber for buttons
        button_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
            Topic::buttons, 1, std::bind(&IoNode::on_button_callback, this, std::placeholders::_1)
        );

        // Initialize the publisher for LEDs
        led_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(Topic::set_rgb_leds, 1);

        // Initialize the publisher of buttons for main node
        button_publisher_ = this->create_publisher<std_msgs::msg::UInt8>(Topic::ionode_buttons, 1);
        kinematics_ = kinematics;
        line_ = line;
    }

    int IoNode::get_button_pressed() const {
        return IoNode::button_pressed_;
    }

    void IoNode::set_led_color(uint8_t led_number, uint8_t R, uint8_t G, uint8_t B) {
        if(led_number >= 3) return; // we have only 3 LEDs on our robot
        std_msgs::msg::UInt8MultiArray leds = std_msgs::msg::UInt8MultiArray();
        switch(led_number) {
            case 0: leds.data = {R, G, B, IoNode::leds[3], IoNode::leds[4], IoNode::leds[5], IoNode::leds[6], IoNode::leds[7], IoNode::leds[8]}; break;
            case 1: leds.data = {IoNode::leds[0], IoNode::leds[1], IoNode::leds[2], R, G, B, IoNode::leds[6], IoNode::leds[7], IoNode::leds[8]}; break;
            case 2: leds.data = {IoNode::leds[0], IoNode::leds[1], IoNode::leds[2], IoNode::leds[3], IoNode::leds[4], IoNode::leds[5], R, G, B}; break;
        }
        led_publisher_->publish(leds);
    }

    void IoNode::set_all_leds_color(uint8_t R, uint8_t G, uint8_t B) {
        std_msgs::msg::UInt8MultiArray leds = std_msgs::msg::UInt8MultiArray();
        leds.data = {R, G, B, R, G, B, R, G , B};
        led_publisher_->publish(leds);
    }

    void IoNode::on_button_callback(std_msgs::msg::UInt8_<std::allocator<void>>::SharedPtr msg) {
        // processing message to button number - main purpose of this function
        IoNode::button_pressed_ = msg->data;

        std_msgs::msg::UInt8 newMsg = std_msgs::msg::UInt8();
        newMsg.data = msg->data;

        button_publisher_->publish(newMsg);

        // TEMPORARY: LAUNCHING EXPERIMENTS VIA BUTTONS
        // --------------------------------------------

        std::cout << msg->data << std::endl;

        if (msg->data == 0) {
            // 1st button + red light
            set_all_leds_color(255, 0, 0);
            line_->calibrationStart();
        } else if (msg->data == 1) {
            // 2nd button + green light
            set_all_leds_color(0, 255, 0);
        } else {
            // 3rd button + blue light
            set_all_leds_color(0, 0, 255);
        }

        // ------------------------
        // END OF TEMPORARY SECTION

    }
}
