#ifndef LINE_NODE_HPP
#define LINE_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include "generalNode.hpp"
#include "kinematics_node.hpp"
#include "io_node.hpp"
#include "pid.hpp"

enum class DiscreteLinePose {
    LineOnLeft,
    LineOnRight,
    LineNone,
    LineBoth,
};

enum class SensorsMode {
    None,
    Calibration,
    FeedbackBang,
    FeedbackPID,
};

namespace nodes
{

    class LineNode : public GeneralNode {
    public:

        LineNode(std::shared_ptr<KinematicsNode> kinematics, std::shared_ptr<nodes::IoNode> ioNode);

        ~LineNode();

        // relative pose to line in meters
        float get_continuous_line_pose() const;

        DiscreteLinePose get_discrete_line_pose() const;
        void estimate_descrete_line_pose(float l_norm, float r_norm);

        uint16_t left_max;
        uint16_t left_min;
        uint16_t right_max;
        uint16_t right_min;
        SensorsMode get_sensors_mode();

        void calibrationStart();
        void stop();
        void calibrationEnd(bool continous);
    private:
        std::shared_ptr<IoNode> ioNode_;
        std::atomic<SensorsMode> mode;
        double left_sensor;
        double right_sensor;
        std::shared_ptr<KinematicsNode> kinematics_;
        void normalize(uint16_t dataLeft, uint16_t dataRight);
        void calibrate(uint16_t dataLeft, uint16_t dataRight);
        rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr line_sensors_subscriber_;
        double normalizeData(uint16_t data, uint16_t min, uint16_t max) const;
        void on_line_sensors_msg(std::shared_ptr<std_msgs::msg::UInt16MultiArray> msg);
        float estimate_continuous_line_pose(float left_value, float right_value);

        algorithms::Pid* algo_;

        // DiscreteLinePose estimate_descrete_line_pose(float l_norm, float r_norm) const;
    };
}

#endif //LINE_NODE_HPP
