#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <cv_bridge/cv_bridge.h>
#include "aruco.hpp"
#include "aruco_node.hpp"
#include "helper.hpp"
// ArucoNode

namespace nodes {

    ArucoNode::ArucoNode(): Node("ArucoNode") {
        // camera_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        //   Topic::camera, 1, std::bind(&ArucoNode::on_camera_msg, this, std::placeholders::_1));
        aruco_detector_ = algorithms::ArucoDetector();
        aruco_publisher_ = this->create_publisher<std_msgs::msg::UInt8>(Topic::aruco, 1);
    }
    void ArucoNode::init(){
        image_transport_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
        camera_subscriber_ = this->image_transport_->subscribe(Topic::camera, 1, std::bind(&ArucoNode::on_camera_msg, this, std::placeholders::_1));
    }
    void ArucoNode::on_camera_msg(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        // convert camera image to opencv format
        cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;

        // run detection
        std::vector <algorithms::ArucoDetector::Aruco> tag = aruco_detector_.detect(image);

        // publish detected aruco tag number
        if(tag.size() != 0) {
            std_msgs::msg::UInt8 newMsg = std_msgs::msg::UInt8();
            newMsg.data = tag.back().id;
            aruco_publisher_->publish(newMsg);
        }
    }

}
