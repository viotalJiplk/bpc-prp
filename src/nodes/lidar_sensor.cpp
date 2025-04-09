//
// Created by root on 4/9/25.
//

#include "lidar_sensor_node.hpp"
#include "helper.hpp"

namespace nodes{
    LidarSensorNode::LidarSensorNode(): rclcpp::Node("LidarNode"){
        lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                    Topic::lidar, 1, std::bind(&LidarSensorNode::lidarCallback, this, std::placeholders::_1));
        lidar_filtered_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(Topic::lidarFiltered, 1);
        algo_ = std::make_shared<algorithms::LidarFiltr>();
    }
    LidarSensorNode::~LidarSensorNode() {

    }
    void LidarSensorNode::lidarCallback(std::shared_ptr<sensor_msgs::msg::LaserScan> msg) {
        algorithms::LidarFiltrResults result = algo_->apply_filter(msg->ranges, msg->angle_min, msg->angle_max);
        std_msgs::msg::Float32MultiArray lidar = std_msgs::msg::Float32MultiArray();
        lidar.data = {result.left, result.front, result.right, result.back};
        lidar_filtered_publisher_->publish(lidar);
    }
}