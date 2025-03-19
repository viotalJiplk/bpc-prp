#include "encoders.hpp"
#include "helper.hpp"

namespace nodes {
    Encoder::Encoder(): GeneralNode("EncoderReader", 2) {
            // Initialize the subscriber
             encoderSubscriber = this->create_subscription<std_msgs::msg::UInt32MultiArray>(
                Topic::encoders, 1, std::bind(&Encoder::on_encoder_callback, this, std::placeholders::_1));
    }
    void Encoder::on_encoder_callback(std_msgs::msg::UInt32MultiArray_<std::allocator<void>>::SharedPtr msg) {
        left.store(msg->data[0]);
        right.store(msg->data[1]);
        publish();
    }

    uint32_t Encoder::getRightEncoderState() {
        return right.load();
    }
    uint32_t Encoder::getLeftEncoderState() {
        return left.load();
    }
    // ...
}
