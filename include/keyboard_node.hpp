//
// Created by root on 4/9/25.
//

#ifndef KEYBOARD_H
#define KEYBOARD_H
#include "rclcpp/rclcpp.hpp"
#include <termios.h>
#include "std_msgs/msg/char.hpp"


namespace nodes {
    class KeyboardInputNode : public rclcpp::Node {
    public:
        // Constructor
        KeyboardInputNode();

        // Destructor (default)
        ~KeyboardInputNode();

    private:
        rclcpp::Publisher<std_msgs::msg::Char>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        struct termios orig_termios_;
        void configure_terminal();
        void reset_terminal();
        void check_keyboard();
    };
}



#endif //KEYBOARD_H
