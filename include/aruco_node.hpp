// aruco_node.hpp
// BPC-PRP project 2025
// xvarec06 & xruzic56
//
// Header file for camera reading and ArUco marker detection node.


#ifndef ARUCO_NODE_HPP
#define ARUCO_NODE_HPP

#include <cstdint>
#include <atomic>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <image_transport/image_transport.hpp>

#include "aruco.hpp"

namespace nodes {

    class ArucoNode : public rclcpp::Node {
    public:
        ArucoNode();
        ~ArucoNode() override = default;
        void init();

    private:
        std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::CompressedImage_<std::allocator<void>>>> camera_subscriber_;

        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr aruco_publisher_;
        std::shared_ptr<image_transport::ImageTransport> image_transport_;

        algorithms::ArucoDetector aruco_detector_;

        void on_camera_msg(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& msg);
    };

}


#endif //ARUCO_NODE_HPP
