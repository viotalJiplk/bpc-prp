#include "main_node.hpp"
#include "helper.hpp"


namespace nodes {
    MainNode::MainNode(
        std::shared_ptr<IoNode> ionode,
        std::shared_ptr<LineNode> line,
        std::shared_ptr<KinematicsNode> kinematics,
        std::shared_ptr<UltrasoundNode> ultrasound,
        std::shared_ptr<KeyboardInputNode> keyboard_input,
        std::shared_ptr<LidarNode> lidar_node
        ): Node("MainNode") {
       line_= line;
       kinematics_ = kinematics;
        ultrasound_ = ultrasound;
        keyboard_input_ = keyboard_input;
        lidar_node_ = lidar_node;
        button_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
        Topic::ionode_buttons, 1, std::bind(&MainNode::button_callback, this, std::placeholders::_1));
        keyboard_subscriber_ = this->create_subscription<std_msgs::msg::Char>(
        Topic::keyboardIn, 1, std::bind(&MainNode::keyboard_callback, this, std::placeholders::_1));
    }

    MainNode::~MainNode() {
        // Destructor Implementation (if needed)
    }

    void MainNode::FollowLine() {
        // TODO
    }

    void MainNode::keyboard_callback(std_msgs::msg::Char_<std::allocator<void>>::SharedPtr msg) {
        switch(msg->data) {
            case 'w':
                this->kinematics_->motorSpeed(10, 10, true, [](bool success){});
                break;
            case 's':
                this->kinematics_->motorSpeed(-10, -10, true, [](bool success){});
                break;
            case 'a':
                this->kinematics_->motorSpeed(-5, 5, true, [](bool success){});
                break;
            case 'd':
                this->kinematics_->motorSpeed(5, -5, true, [](bool success){});
                break;
            case 'l':
                if (this->lidar_node_->get_sensors_mode() == LidarMode::None) {
                    this->lidar_node_->start(true);
                }else {
                    this->lidar_node_->stop();
                }

                break;
            case ' ':
                this->kinematics_->motorSpeed(0, 0, true, [](bool success){});
                break;
        }
    }

    void MainNode::button_callback(std_msgs::msg::UInt8_<std::allocator<void>>::SharedPtr msg) {
        switch(msg->data) {
            case 0:
                if (this->ultrasound_->get_sensors_mode() == UltrasoundMode::None) {
                    this->ultrasound_->calibrationStart();
                    this->ultrasound_->calibrationEnd(true);
                }else {
                    this->ultrasound_->stop();
                }
                break;
            case 1:
                if (this->line_->get_sensors_mode() == SensorsMode::None) {
                    line_->calibrationStart();
                    kinematics_->angle(0.5, 5,  [this](bool result) {
                        this->kinematics_->angle(-1, 5, [this](bool result) {
                            this->kinematics_->angle(0.5, 5,  [this](bool result) {
                                this->line_->calibrationEnd(false);
                                std::cout << "Finished moving forward" << std::endl;
                            });
                        });
                    });
                } else {
                    this->line_->stop();
                }
                break;
            case 2:
                if (this->line_->get_sensors_mode() == SensorsMode::None) {
                    line_->calibrationStart();
                    kinematics_->angle(0.5, 5,  [this](bool result) {
                        this->kinematics_->angle(-1, 5, [this](bool result) {
                            this->kinematics_->angle(0.5, 5,  [this](bool result) {
                                this->line_->calibrationEnd(true);
                                std::cout << "Finished moving forward" << std::endl;
                            });
                        });
                    });
                } else {
                    this->line_->stop();
                }
                break;
            default: break;
        }
    }

    
}
