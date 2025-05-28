// line.cpp
// BPC-PRP project 2025
// xvarec06 & xruzic56
//
// Source file for line estimation and following node.

#include <io_node.hpp>

#include "line_node.hpp"
#include "helper.hpp"
#include "pid.hpp"

struct descrete {
    double lineReadingDeviation;
    double extremeLineReadingDeviation;
    uint16_t maxDifferentialSpeed;
    uint16_t minDifferentialSpeed;
    double angleAngle;
    uint16_t angleSpeed;
    uint16_t forwardSpeed;
};

struct descrete descreteValues = {
    .lineReadingDeviation = 0.05,
    .extremeLineReadingDeviation = 0.2,
    .maxDifferentialSpeed = 8,
    .minDifferentialSpeed = 4,
    .angleAngle = 3,
    .angleSpeed = 3,
    .forwardSpeed = 7,
}; //tested for orange robot

struct pid {
    double kp;
    double ki;
    double kd;
    double mid;
    double deviation;
};

struct pid pidValues = {
    .kp = 1.0,
    .ki = 0.001,
    .kd = 0.1,
    .mid = 2.0,
    .deviation = 4.0,
};

struct untilLine {
    float NormalLeftMin;
    float NormalLeftMax;
    float NormalRightMin;
    float NormalRightMax;
};

struct untilLine untilLineValues = {
    .NormalLeftMin = 0,
    .NormalLeftMax = 65,
    .NormalRightMin = 14,
    .NormalRightMax = 44
};

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
        algo_ = new algorithms::Pid(pidValues.kp, pidValues.ki, pidValues.kd);
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
        ioNode_->set_led_color(1, 255, 255, 0);
        left_min = 1024;
        left_max = 0;
        right_min = 1024;
        right_max = 0;
        mode.store(SensorsMode::Calibration);
    }

    void LineNode::calibrationEnd(bool continous) {
        ioNode_->set_led_color(1, 0, 255, 0);
        if (continous) {
            mode.store(SensorsMode::FeedbackPID);
        }else {
            mode.store(SensorsMode::FeedbackBang);
        }
        std::cout << "Calibrated max: " << left_max << ", " << right_max << std::endl;
        std::cout << "Calibrated min: " << left_min << ", " << right_min << std::endl;
    }

    void LineNode::stop() {
        mode.store(SensorsMode::None);
        ioNode_->set_led_color(1, 255, 0, 0);
    }

    void LineNode::on_line_sensors_msg(std::shared_ptr<std_msgs::msg::UInt16MultiArray> msg){
        if (mode.load() == SensorsMode::Calibration) {
            calibrate(msg->data[0], msg->data[1]);
        } else if (mode.load() == SensorsMode::FeedbackBang) {
            normalize(msg->data[0], msg->data[1]);
            estimate_descrete_line_pose(left_sensor, right_sensor);

        } else if (mode.load() == SensorsMode::FeedbackPID) {
            normalize(msg->data[0], msg->data[1]);
            estimate_continuous_line_pose(left_sensor, right_sensor);
        }else if (mode.load() == SensorsMode::UntilLine) {
            forwardUntilLineCallback(left_sensor, right_sensor);
        }
        publish();
    }

    float LineNode::estimate_continuous_line_pose(float left_value, float right_value){
        double result = left_value - right_value;
        float diff = algo_->step(result, 1);
        if (diff > 1) {
            diff = 1.0;
        }else if (diff < -1) {
            diff = -1.0;
        }
        int leftMotor = (-diff)*pidValues.deviation+pidValues.mid;
        int rightMotor = (diff)*pidValues.deviation+pidValues.mid;
        kinematics_->motorSpeed(leftMotor, rightMotor, [](bool sucess){});
        return 0.0;
    }

    SensorsMode LineNode::get_sensors_mode() {
        return mode.load();
    }

    void LineNode::estimate_descrete_line_pose(float l_norm, float r_norm) {
        float result = l_norm - r_norm;
        if(result > descreteValues.extremeLineReadingDeviation){
            kinematics_->angle(descreteValues.angleAngle, descreteValues.angleSpeed, [](bool sucess){});
        }else if(result < -descreteValues.extremeLineReadingDeviation){
            kinematics_->angle(-descreteValues.angleAngle, descreteValues.angleSpeed, [](bool sucess){});
        }else if(result > descreteValues.lineReadingDeviation){
            kinematics_->motorSpeed(descreteValues.minDifferentialSpeed, descreteValues.maxDifferentialSpeed, [](bool sucess){});
        }else if(result < -descreteValues.lineReadingDeviation){
            kinematics_->motorSpeed(descreteValues.maxDifferentialSpeed, descreteValues.minDifferentialSpeed, [](bool sucess){});
        }else if(l_norm > descreteValues.lineReadingDeviation && r_norm > descreteValues.lineReadingDeviation){
            kinematics_->forward(10, descreteValues.forwardSpeed, [](bool sucess){});
        }else{
            kinematics_->forward(10, descreteValues.forwardSpeed, [](bool sucess){});
        }
    }

    void LineNode::forwardUntilLineCallback(float left_value, float right_value) {
        left_value = this->normalizeData(left_value, untilLineValues.NormalLeftMin, untilLineValues.NormalLeftMax);
        right_value = this->normalizeData(right_value, untilLineValues.NormalRightMin, untilLineValues.NormalRightMax);
        std::cout << left_value << ", " << right_value << std::endl;
        if (left_value > 0.5 or right_value > 0.5)
        {
            this->kinematics_->stop();
            auto callback = this->untilLineCallbackEnd;
            this->untilLineCallbackEnd = [](){};
            callback();
        } 
    }

    void LineNode::forwardUntilLine(std::function<void(void)> callback){
        this->mode.store(SensorsMode::UntilLine);
        this->untilLineCallbackEnd = callback;
        this->kinematics_->motorSpeed(10, 10, true,[](bool sucess){});
    }
}
