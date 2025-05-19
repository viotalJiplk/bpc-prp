#include "io_node.hpp"

namespace nodes {
    IoNode::IoNode(): Node("buttonsReader") {
        // Initialize the subscriber for buttons
        button_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
            Topic::buttons, 1, std::bind(&IoNode::on_button_callback, this, std::placeholders::_1)
        );

        // Initialize the publisher for LEDs
        led_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(Topic::set_rgb_leds, 1);

        // Initialize the publisher of buttons for main node
        button_publisher_ = this->create_publisher<std_msgs::msg::UInt8>(Topic::ionode_buttons, 1);
    }

    int IoNode::get_button_pressed() const {
        return IoNode::button_pressed_;
    }

    void IoNode::set_led_color(uint8_t led_number, uint8_t R, uint8_t G, uint8_t B) {
        if(led_number > 3) return; // we have only 4 LEDs on our robot
        std_msgs::msg::UInt8MultiArray leds = std_msgs::msg::UInt8MultiArray();
        switch(led_number) {
            case 0:
                this->leds[0] = R;
                this->leds[1] = G;
                this->leds[2] = B;
                break;
            case 1:
                this->leds[3] = R;
                this->leds[4] = G;
                this->leds[5] = B;
                break;
            case 2:
                this->leds[6] = R;
                this->leds[7] = G;
                this->leds[8] = B;
                break;
            case 3:
                this->leds[9] = R;
                this->leds[10] = G;
                this->leds[11] = B;
                break;
            default:
                break;
        }
        for (int i = 0 ; i < 12 ; i++)
        {
            leds.data.push_back(this->leds[i]);
        }
        led_publisher_->publish(leds);
    }

    void IoNode::set_all_leds_color(uint8_t R, uint8_t G, uint8_t B) {
        std_msgs::msg::UInt8MultiArray leds = std_msgs::msg::UInt8MultiArray();
        for (int i = 0 ; i < 12 ; i++)
        {
            if (i%3 == 0) this->leds[i] = R;
            else if (i%3 == 1) this->leds[i] = G;
            else if (i%3 == 2) this->leds[i] = B;
        }
        leds.data = {R, G, B, R, G, B, R, G , B, R, G, B};
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

        /*if (msg->data == 0) {
            // 1st button + red light
            set_all_leds_color(255, 0, 0);
            line_->calibrationStart();
        } else if (msg->data == 1) {
            // 2nd button + green light
            set_all_leds_color(0, 255, 0);
        } else {
            // 3rd button + blue light
            set_all_leds_color(0, 0, 255);
        }*/

        // ------------------------
        // END OF TEMPORARY SECTION

    }
    void IoNode::showIntersection(IntersectionType intersection){
        if (intersection == IntersectionType::AllFour){
            this->set_all_leds_color(0, 0, 255);
        }else if (intersection == IntersectionType::TopT){
            this->set_led_color(0, 0, 0, 255);
            this->set_led_color(1, 0, 0, 255);
            this->set_led_color(2, 0, 0, 255);
            this->set_led_color(3, 255, 0, 0);
        }else if (intersection == IntersectionType::LeftT){
            this->set_led_color(0, 0, 0, 255);
            this->set_led_color(1, 255, 0, 0);
            this->set_led_color(2, 0, 0, 255);
            this->set_led_color(3, 0, 0, 255);
        }else if (intersection == IntersectionType::RightT){
            this->set_led_color(0, 0, 0, 255);
            this->set_led_color(1, 0, 0, 255);
            this->set_led_color(2, 255, 0, 0);
            this->set_led_color(3, 0, 0, 255);
        }else if (intersection == IntersectionType::U){
            this->set_led_color(0, 0, 0, 255);
            this->set_led_color(1, 255, 0, 0);
            this->set_led_color(2, 255, 0, 0);
            this->set_led_color(3, 255, 0, 0);
        }else if (intersection == IntersectionType::RightTurn){
            this->set_led_color(0, 0, 0, 255);
            this->set_led_color(1, 0, 0, 255);
            this->set_led_color(2, 255, 0, 0);
            this->set_led_color(3, 255, 0, 255);
        }else if (intersection == IntersectionType::LeftTurn){
            this->set_led_color(0, 0, 0, 255);
            this->set_led_color(1, 255, 0, 0);
            this->set_led_color(2, 0, 0, 255);
            this->set_led_color(3, 255, 0, 255);
        }else{
            this->set_led_color(0, 0, 0, 0);
            this->set_led_color(1, 0, 0, 0);
            this->set_led_color(2, 0, 0, 0);
            this->set_led_color(3, 0, 255, 0);
        }
        // std::cout << "hi" << std::endl;

    }
}
