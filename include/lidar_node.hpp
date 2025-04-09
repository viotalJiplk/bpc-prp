#ifndef LIDAR_NODE_HPP
#define LIDAR_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include "generalNode.hpp"
#include "kinematics_node.hpp"
#include "io_node.hpp"
#include "pid.hpp"
#include "ultrasound_node.hpp"

enum class LidarMode {
    None,
    FeedbackBang,
    FeedbackPID,
};

struct lidarResult{
    double left;
    double front;
    double right;
    double back;
};

namespace nodes
{

    class LidarNode : public GeneralNode {
    public:

        LidarNode(std::shared_ptr<KinematicsNode> kinematics, std::shared_ptr<nodes::IoNode> ioNode);

        ~LidarNode();

        void estimate_descrete_lidar_pose(double l_norm, double r_norm);

        uint8_t left_max;
        uint8_t left_min;
        uint8_t front_max;
        uint8_t front_min;
        uint8_t right_max;
        uint8_t right_min;
        uint8_t back_max;
        uint8_t back_min;
        LidarMode get_sensors_mode();

        void stop();
        void start(bool continous);
    private:
        std::atomic<uint32_t> count_;
        std::shared_ptr<IoNode> ioNode_;
        std::atomic<LidarMode> mode;
        std::shared_ptr<KinematicsNode> kinematics_;
        struct lidarResult normalize(float dataLeft, float dataFront, float dataRight, float dataBack);
        std::atomic<long> prevT_;
        void calibrate(uint8_t dataLeft, uint8_t dataMiddle, uint8_t dataRight);
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr lidar_sensors_subscriber_;
        double normalizeData(float data, float min, float max) const;
        void on_lidar_sensors_msg(std::shared_ptr<std_msgs::msg::Float32MultiArray> msg);

        double estimate_continuous_lidar_pose(double left_value, double front, double right_value, double back);

        algorithms::Pid* algo_;

        // DiscreteLinePose estimate_descrete_ultrasound_pose(float l_norm, float r_norm) const;
    };
}

#endif //LINE_NODE_HPP
