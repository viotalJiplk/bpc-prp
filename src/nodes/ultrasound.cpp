#include <io_node.hpp>

#include "helper.hpp"
#include "pid.hpp"
#include "ultrasound_node.hpp"
#include <chrono>

struct descreteUltrasound {
    double lineReadingDeviation;
    double extremeLineReadingDeviation;
    uint8_t maxDifferentialSpeed;
    uint8_t minDifferentialSpeed;
    double angleAngle;
    uint8_t angleSpeed;
    uint8_t forwardSpeed;
};

struct descreteUltrasound descreteValuesUltrasound = {
    .lineReadingDeviation = 0.05,
    .extremeLineReadingDeviation = 0.2,
    .maxDifferentialSpeed = 8,
    .minDifferentialSpeed = 4,
    .angleAngle = 3,
    .angleSpeed = 3,
    .forwardSpeed = 7,
}; //tested for orange robot

struct pidUltrasound {
    double kp;
    double ki;
    double kd;
    double mid;
    double deviation;
    double error;
    double middleError;
    double leftRightError;
};

struct pidUltrasound pidUltrasoundValues = {
    .kp = 1,
    .ki = 0.00,
    .kd = 0.4,
    .mid = 10.0,
    .deviation = 9.0,
    .error = 0.005,
    .middleError = 0.05,
    .leftRightError = 0.06,
};

struct extreme {
    double middleError;
    double leftRightError;
    double intersection;
};

struct extreme extremeValues = {
    .middleError = 0.2,
    .leftRightError = 0.2,
};

namespace nodes {
    UltrasoundNode::UltrasoundNode(std::shared_ptr<KinematicsNode> kinematics, std::shared_ptr<IoNode> ioNode): GeneralNode("UltrasoundNode", 1) {
        ultrasound_sensors_subscriber_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
                  Topic::ultrasoundFiltered, 1, std::bind(&UltrasoundNode::on_ultrasound_sensors_msg, this, std::placeholders::_1));
        left_min = 0;
        left_max = 50;
        middle_min = 0;
        middle_max = 50;
        right_min = 0;
        right_max = 50;
        ioNode_ = ioNode;
        mode = UltrasoundMode::None;
        kinematics_ = kinematics;
        algo_ = new algorithms::Pid(pidUltrasoundValues.kp, pidUltrasoundValues.ki, pidUltrasoundValues.kd);
        count_.store(0);
        prevT_.store(0);
        previousDirection = UltrasoundDirection::Front;
    }

    UltrasoundNode::~UltrasoundNode() {
        // Destructor Implementation (if needed)
    }

    double UltrasoundNode::normalizeData(uint8_t data, uint8_t min, uint8_t max) const {
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

    struct ultrasoundResult UltrasoundNode::normalize(uint8_t dataLeft, uint8_t dataMiddle, uint8_t dataRight) {
        return {
            .left = normalizeData(dataLeft, left_min, left_max),
            .middle = normalizeData(dataMiddle, middle_min, middle_max),
            .right = normalizeData(dataRight, right_min, right_max),
        };
    }

    void UltrasoundNode::calibrate(uint8_t dataLeft, uint8_t dataMiddle, uint8_t dataRight) {
        if (dataLeft > left_max) {
            left_max = dataLeft;
        }
        if (dataLeft < left_min) {
            left_min = dataLeft;
        }

        if (dataMiddle > left_max) {
            left_max = dataMiddle;
        }
        if (dataMiddle < left_min) {
            left_min = dataMiddle;
        }

        if (dataRight > right_max) {
            right_max = dataRight;
        }
        if (dataRight < right_min) {
            right_min = dataRight;
        }
    }

    void UltrasoundNode::calibrationStart() {
        ioNode_->set_led_color(0, 255, 255, 0);
        left_min = 0;
        left_max = 170;
        middle_min = 0;
        middle_max = 170;
        right_min = 0;
        right_max = 170;
        mode.store(UltrasoundMode::Calibration);
    }

    void UltrasoundNode::calibrationEnd(bool continous) {
        ioNode_->set_led_color(0, 0, 255, 0);
        if (continous) {
            mode.store(UltrasoundMode::FeedbackPID);
        }else {
            mode.store(UltrasoundMode::FeedbackBang);
        }
    }

    void UltrasoundNode::stop() {
        mode.store(UltrasoundMode::None);
        kinematics_->stop();
        ioNode_->set_led_color(0, 255, 0, 0);
    }

    void UltrasoundNode::on_ultrasound_sensors_msg(std::shared_ptr<std_msgs::msg::UInt8MultiArray> msg){
        struct ultrasoundResult normalResult = normalize(msg->data[0], msg->data[1], msg->data[2]);
        if (mode.load() == UltrasoundMode::Calibration) {
            calibrate(msg->data[0], msg->data[1], msg->data[2]);
            /* std::cout << "calibrate:" << std::endl;
            std::cout << "left: " << left_min << "/" << left_max << std::endl;
            std::cout << "right: " << right_min << "/" << right_max << std::endl;*/
        } else if (mode.load() == UltrasoundMode::FeedbackBang) {
            estimate_descrete_ultrasound_pose(normalResult.left, normalResult.right);

        } else if (mode.load() == UltrasoundMode::FeedbackPID) {
            // std::cout << "Raw: " << static_cast<uint32_t>(msg->data[0]) << ", " << static_cast<uint32_t>(msg->data[2]) << std::endl;
            // std::cout << "'Normalized: '" << normalResult.left << ", " << normalResult.right << std::endl;
            estimate_continuous_ultrasound_pose(normalResult.left, normalResult.middle, normalResult.right);
        }else if (mode.load() == UltrasoundMode::ExtremeTesting) {
            extremeTestingCheck(normalResult.left, normalResult.middle, normalResult.right);
        }else if (mode.load() == UltrasoundMode::ExtremeHandling) {
            extremeHandlerCallback(normalResult.left, normalResult.middle, normalResult.right);
        }
    }

    double UltrasoundNode::estimate_continuous_ultrasound_pose(double left_value, double middle, double right_value){
        // std::cout << "Ultrasound values " << left_value << ", " << right_value << std::endl;
        if ((middle > pidUltrasoundValues.middleError) and (left_value > pidUltrasoundValues.leftRightError) and (right_value > pidUltrasoundValues.leftRightError)) {
            previousDirection = UltrasoundDirection::Front;
            auto timeStamp = std::chrono::high_resolution_clock::now();
            long timeNow = std::chrono::duration_cast<std::chrono::milliseconds>(
                timeStamp.time_since_epoch()
            ).count();

            long oldTime = prevT_.exchange(timeNow);

            if (oldTime != 0) {
                double result = left_value - right_value;
                if (abs(result) < pidUltrasoundValues.error) {
                    result = 0.0;
                }
                double dt = (timeNow-oldTime)/1000.0;
                // std::cout << dt << std::endl;
                double diff = algo_->step(result, dt);
                if (diff > 1) {
                    diff = 1.0;
                }else if (diff < -1) {
                    diff = -1.0;
                }
                int leftMotor = (-diff)*pidUltrasoundValues.deviation+pidUltrasoundValues.mid;
                int rightMotor = (diff)*pidUltrasoundValues.deviation+pidUltrasoundValues.mid;
                // std::cout << "Motor settings " << leftMotor << ", " << rightMotor << ", " << result << std::endl;
                kinematics_->motorSpeed(leftMotor, rightMotor, [](bool sucess){});
            }
        } else if (left_value <= pidUltrasoundValues.leftRightError) {
            if (previousDirection == UltrasoundDirection::Left) {
                kinematics_->angle(3, 3, [](bool sucess){});
            }else {
                previousDirection = UltrasoundDirection::Right;
                kinematics_->angle(-3, 3, [](bool sucess){});
            }
        } else if (right_value <= pidUltrasoundValues.leftRightError) {
            if (previousDirection == UltrasoundDirection::Right) {
                kinematics_->angle(-3, 3, [](bool sucess){});
            }else {
                previousDirection = UltrasoundDirection::Left;
                kinematics_->angle(3, 3, [](bool sucess){});
            }
        } else {
            // predict value
            if (previousDirection == UltrasoundDirection::Front){
                if (left_value > right_value) {
                    previousDirection = UltrasoundDirection::Left;
                    kinematics_->angle(3, 3, [](bool sucess){});
                } else {
                    previousDirection = UltrasoundDirection::Right;
                    kinematics_->angle(-3, 3, [](bool sucess){});
                }
            } else if (previousDirection == UltrasoundDirection::Left) {
                kinematics_->angle(3, 3, [](bool sucess){});
            }else {
                previousDirection = UltrasoundDirection::Right;
                kinematics_->angle(-3, 3, [](bool sucess){});
            }
        }
        return 0.0;
    }

    void UltrasoundNode::extremeTestingCheck(double left_value, double middle, double right_value){
        // std::cout << "Ultrasound values " << left_value << ", " << right_value << std::endl;
        if ((middle <= extremeValues.middleError) or (left_value <= extremeValues.leftRightError) or (right_value <= extremeValues.leftRightError)) {
            this->mode.store(UltrasoundMode::None);
            std::function<void(bool)> callback = this->extremeTestingCallback_;
            this->extremeTestingCallback_ = [](bool value){};
            callback(false);
        }
    }

    void UltrasoundNode::extremeHandlerCallback(double left_value, double middle, double right_value){
        // std::cout << "Ultrasound values " << left_value << ", " << right_value << std::endl;
;        if (middle <= extremeValues.middleError) {
            // predict value
            if (previousDirection == UltrasoundDirection::Front){
                if (left_value > right_value) {
                    previousDirection = UltrasoundDirection::Left;
                    kinematics_->angle(3, 3, [](bool sucess){});
                } else {
                    previousDirection = UltrasoundDirection::Right;
                    kinematics_->angle(-3, 3, [](bool sucess){});
                }
            } else if (previousDirection == UltrasoundDirection::Left) {
                kinematics_->angle(3, 3, [](bool sucess){});
            }else {
                previousDirection = UltrasoundDirection::Right;
                kinematics_->angle(-3, 3, [](bool sucess){});
            }
        } else if (left_value <= extremeValues.leftRightError) {
            if (previousDirection == UltrasoundDirection::Left) {
                kinematics_->angle(3, 3, [](bool sucess){});
            }else {
                previousDirection = UltrasoundDirection::Right;
                kinematics_->angle(-3, 3, [](bool sucess){});
            }
        } else if (right_value <= extremeValues.leftRightError) {
            if (previousDirection == UltrasoundDirection::Right) {
                kinematics_->angle(-3, 3, [](bool sucess){});
            }else {
                previousDirection = UltrasoundDirection::Left;
                kinematics_->angle(3, 3, [](bool sucess){});
            }
        } else {
            previousDirection = UltrasoundDirection::Front;
            this->mode.store(UltrasoundMode::None);
            std::function<void()> callback = this->extremeHandleCallback_;
            this->extremeHandleCallback_ = [](){};
            callback();
        }
    }

    void UltrasoundNode::extremeTestingStart(std::function<void(bool)> callback) {
        this->extremeTestingCallback_ = callback;
        this->mode.store(UltrasoundMode::ExtremeTesting);
    };
    void UltrasoundNode::handleExtreme(std::function<void()> callback) {
        this->extremeHandleCallback_ = callback;
        this->mode.store(UltrasoundMode::ExtremeHandling);
    };

    UltrasoundMode UltrasoundNode::get_sensors_mode() {
        return mode.load();
    }

    void UltrasoundNode::estimate_descrete_ultrasound_pose(double l_norm, double r_norm) {
        double result = l_norm - r_norm;
        // std::cout << "result: " <<result  << std::endl;
        if(result > descreteValuesUltrasound.extremeLineReadingDeviation){
            kinematics_->angle(descreteValuesUltrasound.angleAngle, descreteValuesUltrasound.angleSpeed, [](bool sucess){});
        }else if(result < -descreteValuesUltrasound.extremeLineReadingDeviation){
            kinematics_->angle(-descreteValuesUltrasound.angleAngle, descreteValuesUltrasound.angleSpeed, [](bool sucess){});
        }else if(result > descreteValuesUltrasound.lineReadingDeviation){
            kinematics_->motorSpeed(descreteValuesUltrasound.minDifferentialSpeed, descreteValuesUltrasound.maxDifferentialSpeed, [](bool sucess){});
        }else if(result < -descreteValuesUltrasound.lineReadingDeviation){
            kinematics_->motorSpeed(descreteValuesUltrasound.maxDifferentialSpeed, descreteValuesUltrasound.minDifferentialSpeed, [](bool sucess){});
        }else if(l_norm > descreteValuesUltrasound.lineReadingDeviation && r_norm > descreteValuesUltrasound.lineReadingDeviation){
            kinematics_->forward(10, descreteValuesUltrasound.forwardSpeed, [](bool sucess){});
        }else{
            kinematics_->forward(10, descreteValuesUltrasound.forwardSpeed, [](bool sucess){});
        }
    }
}