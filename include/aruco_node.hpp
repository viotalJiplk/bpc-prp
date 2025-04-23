
#ifndef ARUCO_NODE_HPP
#define ARUCO_NODE_HPP

#include <cstdint>
#include <atomic>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/u_int8.hpp>


#include "aruco.hpp"

namespace nodes {

    class ArucoNode : public rclcpp::Node {
    public:
        ArucoNode();
        ~ArucoNode() override = default;

    private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscriber_;

        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr aruco_publisher_;

        algorithms::ArucoDetector aruco_detector_;

        void on_camera_msg(const sensor_msgs::msg::Image::SharedPtr msg);
    };

}


#endif //ARUCO_NODE_HPP
