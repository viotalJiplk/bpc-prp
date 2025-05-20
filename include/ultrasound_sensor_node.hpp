// ultrasound_sensor_node.hpp
// BPC-PRP project 2025
// xvarec06 & xruzic56
//
// Header file for ultrasound sensors data filtering node.


#ifndef ULTRASOUND_SENSOR_NODE_H
#define ULTRASOUND_SENSOR_NODE_H


#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>

#define FIELDLEN 5

namespace nodes
{

    class UltrasoundSensorNode: public rclcpp::Node {
    public:

        UltrasoundSensorNode();

        ~UltrasoundSensorNode();
    private:
        uint32_t count_;
        std::mutex mutex_;
        uint8_t left_[FIELDLEN];
        uint8_t middle_[FIELDLEN];
        uint8_t right_[FIELDLEN];
        void on_ultrasound_sensors_msg(std::shared_ptr<std_msgs::msg::UInt8MultiArray> msg);

        rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr ultrasound_sensors_subscriber_;
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr ultrasound_sensors_filtered_publisher_;
        void FilterMaxMin(std::shared_ptr<std_msgs::msg::UInt8MultiArray> msg);
    };
}

#endif
