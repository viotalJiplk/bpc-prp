#include "main_node.hpp"
#include "helper.hpp"


namespace nodes {
    MainNode::MainNode(
        std::shared_ptr<IoNode> ionode,
        std::shared_ptr<LineNode> line,
        std::shared_ptr<KinematicsNode> kinematics
    ): Node("MainNode") {
       line_= line;
       kinematics_ = kinematics;
       MainNode::button_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
        Topic::ionode_buttons, 1, std::bind(&MainNode::button_callback, this, std::placeholders::_1));
    }

    MainNode::~MainNode() {
        // Destructor Implementation (if needed)
    }

    void MainNode::FollowLine() {
        // TODO
    }

    void MainNode::button_callback(std_msgs::msg::UInt8_<std::allocator<void>>::SharedPtr msg) {
        switch(msg->data) {
            case 0:
                kinematics_->forward(100, [](bool result) {
                    std::cout << "Finished moving forward" << std::endl;
                });
                break;
                //MainNode::FollowLine(); break;
            case 1:
                line_->calibrationStart();
                kinematics_->angle(1, [this](bool result) {
                    this->line_->calibrationEnd();
                    this->kinematics_->angle(-1, [](bool result) {
                        std::cout << "Finished moving forward" << std::endl;
                    });
                });
                break;
            case 2:
                break;
            default: break;
        }
    }

    
}
