// lidar.cpp
// BPC-PRP project 2025
// xvarec06 & xruzic56
//
// Source file for LiDAR data interpretation node.


#include <io_node.hpp>
#include <gtest/internal/gtest-internal.h>

#include "helper.hpp"
#include "pid.hpp"
#include "lidar_node.hpp"

#define PI 3.14159265

struct descreteLidar {
    double lineReadingDeviation;
    double extremeLineReadingDeviation;
    uint8_t maxDifferentialSpeed;
    uint8_t minDifferentialSpeed;
    double angleAngle;
    uint8_t angleSpeed;
    uint8_t forwardSpeed;
};

struct descreteLidar descreteValuesLidar = {
    .lineReadingDeviation = 0.05,
    .extremeLineReadingDeviation = 0.2,
    .maxDifferentialSpeed = 8,
    .minDifferentialSpeed = 4,
    .angleAngle = 3,
    .angleSpeed = 3,
    .forwardSpeed = 7,
}; //tested for orange robot

struct pidLidar {
    double kp;
    double ki;
    double kd;
    double mid;
    double deviation;
    double error;
    double middleError;
    double leftRightError;
    double leftRightMiddleError;
    double intersection;
    double intersectionOut;
    double backStop;
    double extremePreference;
};

struct pidLidar pidLidarValues = {
    .kp = 1,
    .ki = 0.00,
    // .kd = 0.4,
    .kd = 0.0,
    .mid = 14.0,
    .deviation = 5.0,
    .error = 0.005,
    .middleError = 0.04,
    .leftRightError = 0.08,
    .leftRightMiddleError = 0.08,
    .intersection = 0.21,
    .intersectionOut = 0.18,
    .extremePreference = 0.3,
};

namespace nodes {
    LidarNode::LidarNode(std::shared_ptr<KinematicsNode> kinematics, std::shared_ptr<IoNode> ioNode, std::shared_ptr<UltrasoundNode> ultrasoundNode): GeneralNode("LidarNode", 1) {
        lidar_sensors_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
                  Topic::lidarFiltered, 1, std::bind(&LidarNode::on_lidar_sensors_msg, this, std::placeholders::_1));
        left_min = 0.0;
        left_max = 2.5;
        front_min = 0.0;
        front_max = 2.5;
        right_min = 0.0;
        right_max = 2.5;
        back_min = 0.0;
        back_max = 2.5;
        back_left_min = 0.0;
        back_left_max = 2.5;
        back_right_min = 0.0;
        back_right_max = 2.5;
        front_left_min = 0.0;
        front_left_max = 2.5;
        front_right_min = 0.0;
        front_right_max = 2.5;
        ioNode_ = ioNode;
        mode = LidarMode::None;
        kinematics_ = kinematics;
        algo_ = new algorithms::Pid(pidLidarValues.kp, pidLidarValues.ki, pidLidarValues.kd);
        count_.store(0);
        prevT_.store(0);
        ultrasoundNode_ = ultrasoundNode;
        this_intersection_ = IntersectionType::None;
    }

    // alt. constructor
    LidarNode::LidarNode(std::shared_ptr<IoNode> ioNode): GeneralNode("LidarNode", 1) {
        lidar_sensors_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
                  Topic::lidarFiltered, 1, std::bind(&LidarNode::intersectionInfo, this, std::placeholders::_1));
        left_min = 0.0;
        left_max = 2.5;
        front_min = 0.0;
        front_max = 2.5;
        right_min = 0.0;
        right_max = 2.5;
        back_min = 0.0;
        back_max = 2.5;
        back_left_min = 0.0;
        back_left_max = 2.5;
        back_right_min = 0.0;
        back_right_max = 2.5;
        front_left_min = 0.0;
        front_left_max = 2.5;
        front_right_min = 0.0;
        front_right_max = 2.5;
        ioNode_ = ioNode;
        mode = LidarMode::None;
        algo_ = new algorithms::Pid(pidLidarValues.kp, pidLidarValues.ki, pidLidarValues.kd);
        count_.store(0);
        prevT_.store(0);
        this_intersection_ = IntersectionType::None;
    }

    LidarNode::~LidarNode() {
        // Destructor Implementation (if needed)
    }

    double LidarNode::normalizeData(float data, float min, float max) const {
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

    struct lidarResult LidarNode::normalize(float dataFront, float dataFrontLeft, float dataFrontRight, float dataBack, float dataBackLeft,
        float dataBackRight, float dataLeft, float dataRight) {
        return {
            .front = normalizeData(dataFront, front_min, front_max),
            .front_left = normalizeData(dataFrontLeft, front_left_min, front_left_max),
            .front_right = normalizeData(dataFrontRight, front_right_min, front_right_max),
            .back = normalizeData(dataBack, back_min, back_max),
            .back_left = normalizeData(dataBackLeft, back_left_min, back_left_max),
            .back_right = normalizeData(dataBackRight, back_right_min, back_right_max),
            .left = normalizeData(dataLeft, left_min, left_max),
            .right = normalizeData(dataRight, right_min, right_max),
        };
    }

    void LidarNode::start(bool continous, std::function<void(IntersectionType detectedIntersection)> intersection) {
        onIntersection_ = intersection;
        // ioNode_->set_led_color(0, 0, 255, 0);
        prevT_.exchange(0);
        if (continous) {
            mode.store(LidarMode::FeedbackPID);
        }else {
            mode.store(LidarMode::FeedbackBang);
        }
        this->ultrasoundNode_->extremeTestingStart([this](bool intersection) {
            this->onExtreme(intersection);
        });
    }

    void LidarNode::stop() {
        prevT_.exchange(0);
        mode.store(LidarMode::None);
        kinematics_->stop();
        ultrasoundNode_->stop();
        // ioNode_->set_led_color(0, 255, 0, 0);
    }

    void LidarNode::onExtreme(bool intersection) {
        LidarMode activeMode = this->mode.load();
        this->mode.store(LidarMode::None);
        if (intersection) {
            assert(true);
            // this->mode.store(LidarMode::None);
            // std::function<void()> callback = this->onIntersection_;
            // onIntersection_ = [](){};
            // callback();
        } else {
            UltrasoundDirection preferredDirection = UltrasoundDirection::Front;
            lidarResult previousLidar = this->previousLidarResult_.load();
            if ((previousLidar.left > pidLidarValues.extremePreference) and (previousLidar.left > previousLidar.right)) {
                preferredDirection = UltrasoundDirection::Left;
            } else if (previousLidar.right > pidLidarValues.extremePreference) {
                preferredDirection = UltrasoundDirection::Right;
            }

            this->ultrasoundNode_->handleExtreme([this, activeMode]() {
                prevT_.exchange(0);
                this->mode.store(activeMode);
                this->ultrasoundNode_->extremeTestingStart([this](bool intersection) {
                    this->onExtreme(intersection);
                });
            }, preferredDirection);
        }
    }

    void LidarNode::on_lidar_sensors_msg(std::shared_ptr<std_msgs::msg::Float32MultiArray> msg){

        struct lidarResult normalResult = normalize(
                msg->data[0], msg->data[1], msg->data[2], msg->data[3],
                msg->data[4], msg->data[5], msg->data[6], msg->data[7]
            );
        this->previousLidarResult_.store(normalResult);
        if (mode.load() == LidarMode::FeedbackBang) {
            estimate_descrete_lidar_pose(normalResult.left, normalResult.right);

        } else if (mode.load() == LidarMode::FeedbackPID) {
            estimate_continuous_lidar_pose(
                normalResult.left,
                normalResult.front_left,
                normalResult.front,
                normalResult.front_right,
                normalResult.right,
                normalResult.back_right,
                normalResult.back,
                normalResult.back_left
            );
        }else if (mode.load() == LidarMode::Center or mode.load() == LidarMode::CenterLookup) {
            centerHandler(
                normalResult.left,
                normalResult.front_left,
                normalResult.front,
                normalResult.front_right,
                normalResult.right,
                normalResult.back_right,
                normalResult.back,
                normalResult.back_left
            );
        }
    }


    IntersectionType LidarNode::getThisIntersection() {
        lidarResult previousLidar = this->previousLidarResult_.load();
        double valueLeft = previousLidar.left;
        double valueFront = previousLidar.front;
        double valueRight = previousLidar.right;
        double valueBack = previousLidar.back;
        IntersectionType resultType = IntersectionType::None;
        if ((valueLeft > pidLidarValues.intersection) and (valueRight > pidLidarValues.intersection) and (valueFront > pidLidarValues.intersection) and (valueBack > pidLidarValues.intersection))
        {
            resultType = IntersectionType::AllFour;
            this->ioNode_->showIntersection(resultType);
        } else if ((valueRight > pidLidarValues.intersection) and (valueFront > pidLidarValues.intersection) and (valueBack > pidLidarValues.intersection)) {
            resultType = IntersectionType::RightT;
            this->ioNode_->showIntersection(resultType);
        } else if ((valueLeft > pidLidarValues.intersection) and (valueFront > pidLidarValues.intersection) and (valueBack > pidLidarValues.intersection)) {
            resultType = IntersectionType::LeftT;
            this->ioNode_->showIntersection(resultType);
        } else if ((valueLeft > pidLidarValues.intersection) and (valueRight > pidLidarValues.intersection) and (valueBack > pidLidarValues.intersection)) {
            resultType = IntersectionType::TopT;
            this->ioNode_->showIntersection(resultType);
        }else if (valueLeft > pidLidarValues.intersectionOut and valueBack > pidLidarValues.intersectionOut){
            resultType = IntersectionType::LeftTurn;
            this->ioNode_->showIntersection(resultType);
        } else if (valueRight > pidLidarValues.intersectionOut and valueBack > pidLidarValues.intersectionOut)
        {
            resultType = IntersectionType::RightTurn;
            this->ioNode_->showIntersection(resultType);
        }
        this->this_intersection_ = resultType;
        return resultType;
    }

    IntersectionType LidarNode::detectIntersection(double valueLeft, double valueFront, double valueRight, double valueBack) {
        std::cout << valueLeft << ", " << valueFront << ", " << valueRight << ", " << valueBack << std::endl;
        if (this_intersection_ == IntersectionType::None){
            IntersectionType resultType = IntersectionType::None;
            if ((valueLeft > pidLidarValues.intersection) and (valueRight > pidLidarValues.intersection) and (valueFront > pidLidarValues.intersection) and (valueBack > pidLidarValues.intersection))
            {
                resultType = IntersectionType::AllFour;
                this_intersection_ = resultType;
                this->ioNode_->showIntersection(resultType);
            } else if ((valueRight > pidLidarValues.intersection) and (valueFront > pidLidarValues.intersection) and (valueBack > pidLidarValues.intersection)) {
                resultType = IntersectionType::RightT;
                this_intersection_ = resultType;
                this->ioNode_->showIntersection(resultType);
            } else if ((valueLeft > pidLidarValues.intersection) and (valueFront > pidLidarValues.intersection) and (valueBack > pidLidarValues.intersection)) {
                resultType = IntersectionType::LeftT;
                this_intersection_ = resultType;
                this->ioNode_->showIntersection(resultType);
            } else if ((valueLeft > pidLidarValues.intersection) and (valueRight > pidLidarValues.intersection) and (valueBack > pidLidarValues.intersection)) {
                resultType = IntersectionType::TopT;
                this_intersection_ = resultType;
                this->ioNode_->showIntersection(resultType);
            } else if ((valueLeft < pidLidarValues.intersectionOut) and (valueRight < pidLidarValues.intersectionOut) and (valueFront < pidLidarValues.backStop)) {
                resultType = IntersectionType::U;
                this_intersection_ = resultType;
                this->ioNode_->showIntersection(resultType);
            }else if (valueLeft > pidLidarValues.intersectionOut and valueBack > pidLidarValues.intersectionOut)
            {
                resultType = IntersectionType::LeftTurn;
                this_intersection_ = resultType;
                this->ioNode_->showIntersection(resultType);
            } else if (valueRight > pidLidarValues.intersectionOut and valueBack > pidLidarValues.intersectionOut)
            {
                resultType = IntersectionType::RightTurn;
                this_intersection_ = resultType;
                this->ioNode_->showIntersection(resultType);
            }
            return resultType;
        }else{
            if (this_intersection_ == IntersectionType::AllFour and (
                (valueLeft < pidLidarValues.intersectionOut)
                or (valueRight < pidLidarValues.intersectionOut)
                or (valueFront < pidLidarValues.intersectionOut)
                or (valueBack < pidLidarValues.intersectionOut))){
                this_intersection_ = IntersectionType::None;
                this->ioNode_->showIntersection(IntersectionType::None);
            } else if (this_intersection_ == IntersectionType::RightT and (
                (valueRight < pidLidarValues.intersectionOut)
                or (valueFront < pidLidarValues.intersectionOut)
                or (valueBack < pidLidarValues.intersectionOut)
                or (valueLeft > pidLidarValues.intersection))) {
                this->ioNode_->showIntersection(IntersectionType::None);
                this_intersection_ = IntersectionType::None;
            } else if (this_intersection_ == IntersectionType::LeftT and (
                (valueLeft < pidLidarValues.intersectionOut)
                or (valueFront < pidLidarValues.intersectionOut)
                or (valueBack < pidLidarValues.intersectionOut)
                or (valueRight > pidLidarValues.intersection))) {
                this->ioNode_->showIntersection(IntersectionType::None);
                this_intersection_ = IntersectionType::None;
            } else if (this_intersection_ == IntersectionType::TopT and (
                (valueLeft < pidLidarValues.intersectionOut)
                or (valueRight < pidLidarValues.intersectionOut)
                or (valueBack < pidLidarValues.intersectionOut)
                or (valueFront > pidLidarValues.intersection))) {
                this->ioNode_->showIntersection(IntersectionType::None);
                this_intersection_ = IntersectionType::None;
            } else if (this_intersection_ == IntersectionType::LeftTurn and (
                (valueRight > pidLidarValues.intersection)
                or (valueLeft < pidLidarValues.intersectionOut)
                or (valueFront < pidLidarValues.intersectionOut)
                )){
                this->ioNode_->showIntersection(IntersectionType::None);
                this_intersection_ = IntersectionType::None;
            } else if (this_intersection_ == IntersectionType::RightTurn and (
                    (valueLeft > pidLidarValues.intersection)
                    or (valueRight < pidLidarValues.intersectionOut)
                    or (valueFront < pidLidarValues.intersectionOut)
                    )){
                this->ioNode_->showIntersection(IntersectionType::None);
                this_intersection_ = IntersectionType::None;
            }
            return IntersectionType::None;
        }
    }

    void LidarNode::intersectionInfo(std::shared_ptr<std_msgs::msg::Float32MultiArray> msg) {

        struct lidarResult normalResult = normalize(
            msg->data[0], msg->data[1], msg->data[2], msg->data[3],
            msg->data[4], msg->data[5], msg->data[6], msg->data[7]
        );

        // save result for future use
        this->previousLidarResult_.store(normalResult);
        
        IntersectionType resultType = IntersectionType::None;
       
        // ALL FOUR
        if( 
            (normalResult.left  > pidLidarValues.intersection) and 
            (normalResult.right > pidLidarValues.intersection) and 
            (normalResult.front > pidLidarValues.intersection) and 
            (normalResult.back  > pidLidarValues.intersection)
        ) {
            resultType = IntersectionType::AllFour;
        } 
        // RIGHT T
        else if (
            (normalResult.left  < pidLidarValues.intersection) and 
            (normalResult.right > pidLidarValues.intersection) and 
            (normalResult.front > pidLidarValues.intersection) and 
            (normalResult.back  > pidLidarValues.intersection)
        ) {
            resultType = IntersectionType::RightT;
        } 
        // LEFT T
        else if( 
            (normalResult.left  > pidLidarValues.intersection) and 
            (normalResult.right < pidLidarValues.intersection) and 
            (normalResult.front > pidLidarValues.intersection) and 
            (normalResult.back  > pidLidarValues.intersection)
        ) {
            resultType = IntersectionType::LeftT;
        } 
        // TOP T
        else if( 
            (normalResult.left  > pidLidarValues.intersection) and 
            (normalResult.right > pidLidarValues.intersection) and 
            (normalResult.front < pidLidarValues.intersection) and 
            (normalResult.back  > pidLidarValues.intersection)
        ) {
            resultType = IntersectionType::TopT;
        } 
        // U TURN
        else if( 
            (normalResult.left  < pidLidarValues.intersection) and 
            (normalResult.right < pidLidarValues.intersection) and 
            (normalResult.front < pidLidarValues.intersection) and 
            (normalResult.back  > pidLidarValues.intersection)
        ) {
            resultType = IntersectionType::U;
        }
        // LEFT TURN
        else if( 
            (normalResult.left  > pidLidarValues.intersection) and 
            (normalResult.right < pidLidarValues.intersection) and 
            (normalResult.front < pidLidarValues.intersection) and 
            (normalResult.back  > pidLidarValues.intersection)
        ) {
            resultType = IntersectionType::LeftTurn;
        } 
        // RIGHT TURN
        else if( 
            (normalResult.left  < pidLidarValues.intersection) and 
            (normalResult.right > pidLidarValues.intersection) and 
            (normalResult.front < pidLidarValues.intersection) and 
            (normalResult.back  > pidLidarValues.intersection)
        ) {
            resultType = IntersectionType::RightTurn;
        }
        // I (SIMPLE CORRIDOR)
        else if( 
            (normalResult.left  < pidLidarValues.intersection) and 
            (normalResult.right < pidLidarValues.intersection) and 
            (normalResult.front > pidLidarValues.intersection) and 
            (normalResult.back  > pidLidarValues.intersection)
        ) {
            resultType = IntersectionType::RightTurn;
        }
        // (else type stays on value "None")

        // save detected intersection type
        this_intersection_ = resultType;

        // show intersection on LEDs
        this->ioNode_->showIntersection(resultType);
    }

    IntersectionType LidarNode::getIntersectionInfo() {
        return this->this_intersection_;
    }
 

    double LidarNode::estimate_continuous_lidar_pose(double valueLeft, double valueFrontLeft, double valueFront, double valueFrontRight, double valueRight, 
        double valueBackRight, double valueBack, double valueBackLeft) {
        long timeNow = helper::getTimestamp();
        long oldTime = prevT_.exchange(timeNow);
        IntersectionType detectedIntersection = detectIntersection(valueLeft, valueFront, valueRight, valueBack);
        if ((detectedIntersection != IntersectionType::None)) {

            std::function<void(IntersectionType detectedIntersection)> callback = this->onIntersection_;
            this->onIntersection_ = [](IntersectionType detectedIntersection){};
            this->stop();
            callback(detectedIntersection);
            prevT_.exchange(0);
        } else {
            if (oldTime != 0) {
                double result = valueFrontLeft - valueFrontRight;
                if (abs(result) < pidLidarValues.error) {
                    result = 0.0;
                }
                double dt = (timeNow-oldTime)/1000.0;
                double diff = algo_->step(result, dt);
                if (diff > 1) {
                    diff = 1.0;
                }else if (diff < -1) {
                    diff = -1.0;
                }
                int leftMotor = (-diff)*pidLidarValues.deviation+pidLidarValues.mid;
                int rightMotor = (diff)*pidLidarValues.deviation+pidLidarValues.mid;
                kinematics_->motorSpeed(leftMotor, rightMotor, [](bool sucess){});
            }
        }
        return 0.0;
    }

    void LidarNode::centerHandler(double valueLeft, double valueFrontLeft, double valueFront, double valueFrontRight, double valueRight,
        double valueBackRight, double valueBack, double valueBackLeft) {
        double actual = abs(valueBackRight + valueBackLeft + valueFrontRight + valueFrontLeft);
        if (mode.load() == LidarMode::Center) {
            double diff = abs(actual - this->centerMin);
            if (diff  < 0.01) {
                std::function<void()> callback = this->centerCallback_;
                this->centerCallback_ = [](){};
                this->kinematics_->stop();
                this->stop();
                callback();
            }
        }else {
            if (actual < this->centerMin) {
                this->centerMin = actual;
            }
        }
    }

    void LidarNode::center(std::function<void()> after) {
        // does not work
        mode.store(LidarMode::CenterLookup);
        centerMin = std::numeric_limits<double>::max();
        this->centerCallback_ = after;
        this->kinematics_->angle(PI/4, 3, [this](bool success) {
            this->kinematics_->angle(-PI/2, 3, [this](bool success) {
                // std::cout << "Center Min " << this->centerMin << std::endl;
                mode.store(LidarMode::Center);
                this->kinematics_->angle(PI/2, 3, [this](bool success) {
                    if (success) {
                        std::function<void()> callback = this->centerCallback_;
                        this->centerCallback_ = [](){};
                        this->stop();
                        callback();
                    }
                });
            });
        });
    }

    LidarMode LidarNode::get_sensors_mode() {
        return mode.load();
    }

    void LidarNode::estimate_descrete_lidar_pose(double l_norm, double r_norm) {
        double result = l_norm - r_norm;
        if(result > descreteValuesLidar.extremeLineReadingDeviation){
            kinematics_->angle(descreteValuesLidar.angleAngle, descreteValuesLidar.angleSpeed, [](bool sucess){});
        }else if(result < -descreteValuesLidar.extremeLineReadingDeviation){
            kinematics_->angle(-descreteValuesLidar.angleAngle, descreteValuesLidar.angleSpeed, [](bool sucess){});
        }else if(result > descreteValuesLidar.lineReadingDeviation){
            kinematics_->motorSpeed(descreteValuesLidar.minDifferentialSpeed, descreteValuesLidar.maxDifferentialSpeed, [](bool sucess){});
        }else if(result < -descreteValuesLidar.lineReadingDeviation){
            kinematics_->motorSpeed(descreteValuesLidar.maxDifferentialSpeed, descreteValuesLidar.minDifferentialSpeed, [](bool sucess){});
        }else if(l_norm > descreteValuesLidar.lineReadingDeviation && r_norm > descreteValuesLidar.lineReadingDeviation){
            kinematics_->forward(10, descreteValuesLidar.forwardSpeed, [](bool sucess){});
        }else{
            kinematics_->forward(10, descreteValuesLidar.forwardSpeed, [](bool sucess){});
        }
    }
}
