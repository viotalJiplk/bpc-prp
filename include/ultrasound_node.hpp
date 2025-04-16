#ifndef UlLTRASOUND_NODE_HPP
#define UlLTRASOUND_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include "generalNode.hpp"
#include "kinematics_node.hpp"
#include "io_node.hpp"
#include "pid.hpp"
#include "ultrasound_node.hpp"

enum class UltrasoundMode {
    None,
    Calibration,
    ExtremeHandling,
    FeedbackBang,
    FeedbackPID,
    ExtremeTesting,
};

struct ultrasoundResult{
    double left;
    double middle;
    double right;
};

enum class UltrasoundDirection {
    Front,
    Left,
    Right
};

namespace nodes
{
    class UltrasoundNode : public GeneralNode {
    public:

        UltrasoundNode(std::shared_ptr<KinematicsNode> kinematics, std::shared_ptr<nodes::IoNode> ioNode);

        ~UltrasoundNode();

        void estimate_descrete_ultrasound_pose(double l_norm, double r_norm);

        uint8_t left_max;
        uint8_t left_min;
        uint8_t middle_max;
        uint8_t middle_min;
        uint8_t right_max;
        uint8_t right_min;
        UltrasoundMode get_sensors_mode();

        void calibrationStart();
        void stop();
        void calibrationEnd(bool continous);
        void extremeTestingStart(std::function<void(bool)> callback);
        void handleExtreme(std::function<void()> callback);
    private:
        std::function<void(bool)> extremeTestingCallback_;
        std::function<void()> extremeHandleCallback_;
        UltrasoundDirection previousDirection;
        std::atomic<uint32_t> count_;
        std::shared_ptr<IoNode> ioNode_;
        std::atomic<UltrasoundMode> mode;
        std::shared_ptr<KinematicsNode> kinematics_;
        void extremeTestingCheck(double left_value, double middle, double right_value);
        void extremeHandlerCallback(double left_value, double middle, double right_value);
        struct ultrasoundResult normalize(uint8_t dataLeft, uint8_t dataMiddle, uint8_t dataRight);
        std::atomic<long> prevT_;
        void calibrate(uint8_t dataLeft, uint8_t dataMiddle, uint8_t dataRight);
        rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr ultrasound_sensors_subscriber_;
        double normalizeData(uint8_t data, uint8_t min, uint8_t max) const;
        void on_ultrasound_sensors_msg(std::shared_ptr<std_msgs::msg::UInt8MultiArray> msg);

        double estimate_continuous_ultrasound_pose(double left_value, double middle, double right_value);

        algorithms::Pid* algo_;

        // DiscreteLinePose estimate_descrete_ultrasound_pose(float l_norm, float r_norm) const;
    };
}

#endif //LINE_NODE_HPP
