//
// Created by student on 19.3.25.
//

#include "main_node.hpp"
#include "helper.hpp"


namespace nodes {
    MainNode::MainNode(): Node("MainNode") {
        subscriber = this->create_subscription<std_msgs::msg::UInt16>(
                Topic::mainNode, 1, std::bind(&MainNode::on_event, this, std::placeholders::_1));
    }

    MainNode::~MainNode() {
        // Destructor Implementation (if needed)
    }
    void MainNode::on_event(std_msgs::msg::UInt16_<std::allocator<void>>::SharedPtr msg) {
      std::cout << "Received event: " << msg->data << std::endl;
    }
}