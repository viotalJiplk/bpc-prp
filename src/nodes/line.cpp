#include <io_node.hpp>

#include "line_node.hpp"
#include "helper.hpp"

#define LINE_READING_DEVIATION 0.2
#define EXTREME_LINE_READING_DEVIATION 0.4

namespace nodes {
    LineNode::LineNode(std::shared_ptr<KinematicsNode> kinematics, std::shared_ptr<IoNode> ioNode): GeneralNode("LineNode", 1) {
        line_sensors_subscriber_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
                  Topic::line_sensors, 1, std::bind(&LineNode::on_line_sensors_msg, this, std::placeholders::_1));
        left_min = 0;
        left_max = 1024;
        right_min = 0;
        right_max = 1024;
        ioNode_ = ioNode;
        mode = SensorsMode::None;
        kinematics_ = kinematics;
        mainPublisher_ = this->create_publisher<std_msgs::msg::UInt16>(Topic::mainNode, 1);
    }

    LineNode::~LineNode() {
        // Destructor Implementation (if needed)
    }

    float LineNode::get_continuous_line_pose() const{
      return 0.0;
    }
    DiscreteLinePose LineNode::get_discrete_line_pose() const{
        return DiscreteLinePose::LineNone;
    }

    double LineNode::normalizeData(uint16_t data, uint16_t min, uint16_t max) const {
        if (data < min) {
            return 0.0;
        } else if (data > max) {
            return 1.0;
        } else {
            double tmp = (data - min);
            double tmp2 = (max - min);
            return tmp/tmp2;
        }
    }

    void LineNode::normalize(uint16_t dataLeft, uint16_t dataRight) {
        left_sensor = normalizeData(dataLeft, left_min, left_max);
        right_sensor =  normalizeData(dataRight, right_min, right_max);
    }

    void LineNode::calibrate(uint16_t dataLeft, uint16_t dataRight) {
        if (dataLeft > left_max) {
            left_max = dataLeft;
        }
        if (dataLeft < left_min) {
            left_min = dataLeft;
        }

        if (dataRight > right_max) {
            right_max = dataRight;
        }
        if (dataRight < right_min) {
            right_min = dataRight;
        }
    }

    void LineNode::calibrationStart() {
        ioNode_->set_all_leds_color(255, 255, 0);
        left_min = 1024;
        left_max = 0;
        right_min = 1024;
        right_max = 0;
        mode = SensorsMode::Calibration;
    }

    void LineNode::calibrationEnd() {
        ioNode_->set_all_leds_color(0, 255, 0);
        mode = SensorsMode::Feedback;
        std::cout << "Calibrated max: " << left_max << ", " << right_max << std::endl;
        std::cout << "Calibrated min: " << left_min << ", " << right_min << std::endl;
    }

    void LineNode::stop() {
        mode = SensorsMode::None;
        ioNode_->set_all_leds_color(255, 0, 0);
    }

    void LineNode::on_line_sensors_msg(std::shared_ptr<std_msgs::msg::UInt16MultiArray> msg){
        if (mode == SensorsMode::Calibration) {
            calibrate(msg->data[0], msg->data[1]);
            /* std::cout << "calibrate:" << std::endl;
            std::cout << "left: " << left_min << "/" << left_max << std::endl;
            std::cout << "right: " << right_min << "/" << right_max << std::endl;*/
        } else if (mode == SensorsMode::Feedback) {
            normalize(msg->data[0], msg->data[1]);
            estimate_descrete_line_pose(left_sensor, right_sensor);

        }
        publish();
    }

    float LineNode::estimate_continuous_line_pose(float left_value, float right_value){
        return 0.0;
    }

    SensorsMode LineNode::get_sensors_mode() {
        return mode;
    }

    void LineNode::estimate_descrete_line_pose(float l_norm, float r_norm) {
        float result = l_norm - r_norm;
        if(result > EXTREME_LINE_READING_DEVIATION){
            kinematics_->angle(1, 2, [](bool sucess){});
        }else if(result < -EXTREME_LINE_READING_DEVIATION){
            kinematics_->angle(-1, 2, [](bool sucess){});
        }else if(result > LINE_READING_DEVIATION){
            kinematics_->angle(1, 5, [](bool sucess){});
        }else if(result < -LINE_READING_DEVIATION){
            kinematics_->angle(-1, 5, [](bool sucess){});
        }else if(l_norm > LINE_READING_DEVIATION && r_norm > LINE_READING_DEVIATION){
            kinematics_->motorSpeed();
        }else{
            kinematics_->forward(0, 10, [](bool sucess){});
        }
    }
}