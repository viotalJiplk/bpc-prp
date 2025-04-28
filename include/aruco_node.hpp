
#ifndef ARUCO_NODE_HPP
#define ARUCO_NODE_HPP

#include <cstdint>
#include <atomic>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
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
        image_transport::Subscriber camera_subscriber_;

        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr aruco_publisher_;
        std::shared_ptr<image_transport::ImageTransport> image_transport_;

        algorithms::ArucoDetector aruco_detector_;

        void on_camera_msg(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    };

}


#endif //ARUCO_NODE_HPP
