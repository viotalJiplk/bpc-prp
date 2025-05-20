// keyboard_node.hpp
// BPC-PRP project 2025
// xvarec06 & xruzic56
//
// Header file for keyboard input enapsilating node.


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
