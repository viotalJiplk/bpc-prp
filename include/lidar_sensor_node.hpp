//
// Created by root on 4/9/25.
//

#ifndef LIDAR_SENSOR_H
#define LIDAR_SENSOR_H
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "lidar.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>

namespace nodes {
    class LidarSensorNode : public rclcpp::Node {
        public:
            LidarSensorNode();
            ~LidarSensorNode();
        private:
            void lidarCallback(std::shared_ptr<sensor_msgs::msg::LaserScan> msg);
            std::shared_ptr<algorithms::LidarFiltr> algo_;
            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
            rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr lidar_filtered_publisher_;
            std::atomic<long> prevT_;
    };
};



#endif //LIDAR_H
