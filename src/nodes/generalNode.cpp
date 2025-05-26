// generalNode.cpp
// BPC-PRP project 2025
// xvarec06 & xruzic56
//
// Source file for nodes inheritance attempt - general node.


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
