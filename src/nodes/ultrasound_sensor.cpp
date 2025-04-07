//
// Created by root on 4/7/25.
//

#include "ultrasound_sensor_node.hpp"

#include <io_node.hpp>

#include "ultrasound_sensor_node.hpp"
#include "helper.hpp"

namespace nodes {
    UltrasoundSensorNode::UltrasoundSensorNode(): rclcpp::Node("UltrasoundSensorNode"){
        count_ = 0;
        ultrasound_sensors_subscriber_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
                  Topic::ultrasound, 1, std::bind(&UltrasoundSensorNode::on_ultrasound_sensors_msg, this, std::placeholders::_1));
        ultrasound_sensors_filtered_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(Topic::ultrasoundFiltered, 1);
    }

    UltrasoundSensorNode::~UltrasoundSensorNode() {
        // Destructor Implementation (if needed)
    }

    void UltrasoundSensorNode::on_ultrasound_sensors_msg(std::shared_ptr<std_msgs::msg::UInt8MultiArray> msg){
        FilterMaxMin(msg);
        // std_msgs::msg::UInt8MultiArray filtered_msg = std_msgs::msg::UInt8MultiArray();
        // filtered_msg.data = msg->data;
        // ultrasound_sensors_filtered_publisher_->publish(filtered_msg);
    }
    void UltrasoundSensorNode::FilterMaxMin(std::shared_ptr<std_msgs::msg::UInt8MultiArray> msg){
        mutex_.lock();
        if(count_ == FIELDLEN) {
            count_ = 0;
        }

        std::cout << static_cast<uint32_t>(msg->data[0]) << ", " << static_cast<uint32_t>(msg->data[1]) << ", " << static_cast<uint32_t>(msg->data[2])  << std::endl;

        left_[count_] = msg->data[0];
        middle_[count_] = msg->data[1];
        right_[count_] = msg->data[2];
        count_++;

        std_msgs::msg::UInt8MultiArray filtered_msg = std_msgs::msg::UInt8MultiArray();
        uint8_t leftMax = left_[0];
        uint8_t rightMax = right_[0];
        uint8_t middleMax = middle_[0];
        uint8_t leftMin = left_[0];
        uint8_t rightMin = right_[0];
        uint8_t middleMin = middle_[0];

        uint32_t leftSum = left_[0];
        uint32_t rightSum = right_[0];
        uint32_t middleSum = middle_[0];
        for(int i = 1; i < FIELDLEN; i++){
            leftSum += left_[i];
            middleSum += middle_[i];
            rightSum += right_[i];
            if(left_[i] > leftMax){
                leftMax = left_[i];
            }
            if(right_[i] > rightMax){
                rightMax = right_[i];
            }
            if(middle_[i] > middleMax){
                middleMax = middle_[i];
            }
            if(left_[i] > leftMin){
                leftMin = left_[i];
            }
            if(right_[i] > rightMin){
                rightMin = right_[i];
            }
            if(middle_[i] > middleMax){
                middleMax = middle_[i];
            }
        }
        leftSum -= leftMax;
        rightSum -= rightMax;
        middleSum -= middleMax;

        filtered_msg.data = {leftSum/(FIELDLEN-1), middleSum/(FIELDLEN-1),rightSum/(FIELDLEN-1)};

        ultrasound_sensors_filtered_publisher_->publish(filtered_msg);
        mutex_.unlock();
    }
}
