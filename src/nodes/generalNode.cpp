//
// Created by student on 19.3.25.
//

#include "generalNode.hpp"
#include "helper.hpp"

namespace nodes {
    GeneralNode::GeneralNode(std::string nodeName, uint16_t topicId): Node(nodeName) {
        mainPublisher_ = this->create_publisher<std_msgs::msg::UInt16>(Topic::mainNode, 1);
        topicId_.data = topicId;
    }

    GeneralNode::~GeneralNode() {
        // Destructor Implementation (if needed)
    }

    void GeneralNode::publish() {
        mainPublisher_->publish(topicId_);
    }
}