// keyboard.cpp
// BPC-PRP project 2025
// xvarec06 & xruzic56
//
// Source file for keyboard input encapsulating node.


#include "keyboard_node.hpp"


#include "rclcpp/rclcpp.hpp"
#include <unistd.h>
#include <fcntl.h>
#include <helper.hpp>

namespace nodes {
        KeyboardInputNode::KeyboardInputNode() : Node("keyboard_input_node"){
            publisher_ = this->create_publisher<std_msgs::msg::Char>(Topic::keyboardIn, 10);
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&KeyboardInputNode::check_keyboard, this));
            configure_terminal();
            RCLCPP_INFO(this->get_logger(), "Keyboard input node started (press q to quit)");
        }

        KeyboardInputNode::~KeyboardInputNode(){
            reset_terminal();
        }



        void KeyboardInputNode::configure_terminal(){
            if (!isatty(STDIN_FILENO)) {
                RCLCPP_WARN(this->get_logger(), "STDIN is not a TTY — keyboard input might not work!");
            }

            tcgetattr(STDIN_FILENO, &orig_termios_);
            struct termios new_termios = orig_termios_;
            new_termios.c_lflag &= ~(ICANON | ECHO); // Disable canonical mode and echo
            tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);

            // Set non-blocking read
            int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
            fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
        }

        void KeyboardInputNode::reset_terminal(){
            tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios_);
        }

        void KeyboardInputNode::check_keyboard(){
            char c;
            while (read(STDIN_FILENO, &c, 1) == 1)
            {
                std_msgs::msg::Char msg;
                msg.data = c;
                publisher_->publish(msg);

                if (c == 'q')
                {
                    rclcpp::shutdown(); // Quit on 'q'
                }
            }
        }
}
