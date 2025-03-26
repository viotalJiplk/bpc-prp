#include "encoders.hpp"
#include "helper.hpp"

namespace nodes {
    Encoder::Encoder(): GeneralNode("EncoderReader", 2) {
            // Initialize the subscriber
             encoderSubscriber_ = this->create_subscription<std_msgs::msg::UInt32MultiArray>(
                Topic::encoders, 1, std::bind(&Encoder::on_encoder_callback, this, std::placeholders::_1));
             encoderPublisher_ = this->create_publisher<std_msgs::msg::UInt32MultiArray>(Topic::encoders_publisher, 1);

    }
    void Encoder::on_encoder_callback(std_msgs::msg::UInt32MultiArray_<std::allocator<void>>::SharedPtr msg) {
        left.store(msg->data[0]);
        right.store(msg->data[1]);
        publish();
        // TODO add filtration
        auto encoders = std_msgs::msg::UInt32MultiArray();
        encoders.data = {left.load(), right.load()};
        encoderPublisher_->publish(encoders);
    }

    uint32_t Encoder::getRightEncoderState() {
        return right.load();
    }
    uint32_t Encoder::getLeftEncoderState() {
        return left.load();
    }
    // ...
}
