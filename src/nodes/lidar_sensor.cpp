// lidar_sensor.cpp
// BPC-PRP project 2025
// xvarec06 & xruzic56
//
// Source file for LiDAR data filtering node.


#include "lidar_sensor_node.hpp"
#include "helper.hpp"

namespace nodes{
    LidarSensorNode::LidarSensorNode(): rclcpp::Node("LidarNode"){
        lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                    Topic::lidar, 1, std::bind(&LidarSensorNode::lidarCallback, this, std::placeholders::_1));
        lidar_filtered_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(Topic::lidarFiltered, 1);
        algo_ = std::make_shared<algorithms::LidarFiltr>();
        prevT_.store(0);
    }
    LidarSensorNode::~LidarSensorNode() {

    }
    void LidarSensorNode::lidarCallback(std::shared_ptr<sensor_msgs::msg::LaserScan> msg) {
        long timeNow = helper::getTimestamp();
        long oldTime = prevT_.exchange(timeNow);
        if (((timeNow - oldTime) > 700) and (oldTime != 0)) {
            std::cout << "\033[31m time between lidar inputs is too large: " << timeNow - oldTime << " ms\033[0m" << std::endl;
        }
        algorithms::LidarFiltrResults result = algo_->apply_filter(msg->ranges, msg->angle_min, msg->angle_max);
        std_msgs::msg::Float32MultiArray lidar = std_msgs::msg::Float32MultiArray();
        lidar.data = {
            result.front,
            result.front_left,
            result.front_right,
            result.back,
            result.back_left,
            result.back_right,
            result.left,
            result.right
        };
        lidar_filtered_publisher_->publish(lidar);
    }
}
