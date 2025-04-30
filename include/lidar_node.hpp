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
    Center,
    CenterLookup,
};


enum class IntersectionType {
    None,
    LeftT,
    RightT,
    TopT,
    AllFour,
};

enum class PreviousDirection {
    Left,
    Right,
};

struct lidarResult{
    double front;
    double front_left;
    double front_right;
    double back;
    double back_left;
    double back_right;
    double left;
    double right;
};

namespace nodes
{

    class LidarNode : public GeneralNode {
    public:

        LidarNode(std::shared_ptr<KinematicsNode> kinematics, std::shared_ptr<IoNode> ioNode, std::shared_ptr<UltrasoundNode> ultrasoundNode);

        ~LidarNode();

        void estimate_descrete_lidar_pose(double l_norm, double r_norm);

        float left_max;
        float left_min;
        float front_max;
        float front_min;
        float right_max;
        float right_min;
        float back_max;
        float back_min;
        float back_left_min;
        float back_left_max;
        float back_right_min;
        float back_right_max;
        float front_left_min;
        float front_left_max;
        float front_right_min;
        float front_right_max;
        LidarMode get_sensors_mode();

        void stop();
        void start(bool continous, std::function<void(IntersectionType detectedIntersection)> intersection);
        void center(std::function<void()> after);
    private:
        PreviousDirection previous_direction_;
        std::atomic<uint32_t> count_;
        std::shared_ptr<IoNode> ioNode_;
        std::atomic<LidarMode> mode;
        std::shared_ptr<KinematicsNode> kinematics_;
        struct lidarResult normalize(float dataLeft, float dataFrontLeft, float dataFront, float dataFrontRight, float dataRight, 
            float dataBackRight, float dataBack, float dataBackLeft);
        std::atomic<long> prevT_;
        std::atomic<lidarResult> previousLidarResult_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr lidar_sensors_subscriber_;
        double normalizeData(float data, float min, float max) const;
        void on_lidar_sensors_msg(std::shared_ptr<std_msgs::msg::Float32MultiArray> msg);
        void onExtreme(bool);
        std::function<void(IntersectionType detectedIntersection)> onIntersection_;

        void centerHandler(double valueLeft, double valueFrontLeft, double valueFront, double valueFrontRight, double valueRight,
        double valueBackRight, double valueBack, double valueBackLeft);
        std::function<void()> centerCallback_;
        bool inIntersection_;
        IntersectionType detectIntersection(double valueLeft, double valueFront, double valueRight, double valueBack);

        double estimate_continuous_lidar_pose(double valueLeft, double valueFrontLeft, double valueFront, double valueFrontRight, double valueRight, 
            double valueBackRight, double valueBack, double valueBackLeft);

        algorithms::Pid* algo_;
        std::shared_ptr<UltrasoundNode> ultrasoundNode_;
        double centerMin;

        // DiscreteLinePose estimate_descrete_ultrasound_pose(float l_norm, float r_norm) const;
    };
}

#endif //LINE_NODE_HPP
