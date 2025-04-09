#include <io_node.hpp>

#include "helper.hpp"
#include "pid.hpp"
#include "lidar_node.hpp"
#include <chrono>

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
};

struct pidLidar pidLidarValues = {
    .kp = 1,
    .ki = 0.00,
    .kd = 0.7,
    .mid = 10.0,
    .deviation = 10.0,
    .error = 0.005,
    .middleError = 0.04,
    .leftRightError = 0.08,
    .leftRightMiddleError = 0.08,
};

namespace nodes {
    LidarNode::LidarNode(std::shared_ptr<KinematicsNode> kinematics, std::shared_ptr<IoNode> ioNode): GeneralNode("LidarNode", 1) {
        lidar_sensors_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
                  Topic::lidarFiltered, 1, std::bind(&LidarNode::on_lidar_sensors_msg, this, std::placeholders::_1));
        left_min = 0.0;
        left_max = 3.0;
        front_min = 0.0;
        front_max = 3.0;
        right_min = 0.0;
        right_max = 3.0;
        back_min = 0.0;
        back_max = 3.0;
        back_left_min = 0.0;
        back_left_max = 3.0;
        back_right_min = 0.0;
        back_right_max = 3.0;
        front_left_min = 0.0;
        front_left_max = 3.0;
        front_right_min = 0.0;
        front_right_max = 3.0;
        ioNode_ = ioNode;
        mode = LidarMode::None;
        kinematics_ = kinematics;
        algo_ = new algorithms::Pid(pidLidarValues.kp, pidLidarValues.ki, pidLidarValues.kd);
        count_.store(0);
        prevT_.store(0);
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

    struct lidarResult LidarNode::normalize(float dataLeft, float dataFrontLeft, float dataFront, float dataFrontRight, float dataRight, 
        float dataBackRight, float dataBack, float dataBackLeft) {
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

    void LidarNode::start(bool continous) {
        ioNode_->set_led_color(0, 0, 255, 0);
        if (continous) {
            mode.store(LidarMode::FeedbackPID);
        }else {
            mode.store(LidarMode::FeedbackBang);
        }
    }

    void LidarNode::stop() {
        mode.store(LidarMode::None);
        kinematics_->motorSpeed(0,0, [](bool sucess){});
        ioNode_->set_led_color(0, 255, 0, 0);
    }

    void LidarNode::on_lidar_sensors_msg(std::shared_ptr<std_msgs::msg::Float32MultiArray> msg){
        if (mode.load() == LidarMode::FeedbackBang) {
            struct lidarResult normalResult = normalize(
                msg->data[0], msg->data[1], msg->data[2], msg->data[3], 
                msg->data[4], msg->data[5], msg->data[6], msg->data[7]
            );
            estimate_descrete_lidar_pose(normalResult.left, normalResult.right);

        } else if (mode.load() == LidarMode::FeedbackPID) {
            // std::cout << "Raw: " << static_cast<uint32_t>(msg->data[0]) << ", " << static_cast<uint32_t>(msg->data[2]) << std::endl;
            struct lidarResult normalResult = normalize(
                msg->data[0], msg->data[1], msg->data[2], msg->data[3],
                msg->data[4], msg->data[5], msg->data[6], msg->data[7]
            );
            // std::cout << "'Normalized: '" << normalResult.left << ", " << normalResult.right << std::endl;
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
        }
    }

    double LidarNode::estimate_continuous_lidar_pose(double valueLeft, double valueFrontLeft, double valueFront, double valueFrontRight, double valueRight, 
        double valueBackRight, double valueBack, double valueBackLeft) {
        // std::cout << "Lidar values " << left_value << ", " << right_value << std::endl;
        auto timeStamp = std::chrono::high_resolution_clock::now();
        long timeNow = std::chrono::duration_cast<std::chrono::milliseconds>(
            timeStamp.time_since_epoch()
        ).count();

        long oldTime = prevT_.exchange(timeNow);

        if (oldTime != 0) {
            double result = valueLeft - valueRight;
            if (abs(result) < pidLidarValues.error) {
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
            int leftMotor = (-diff)*pidLidarValues.deviation+pidLidarValues.mid;
            int rightMotor = (diff)*pidLidarValues.deviation+pidLidarValues.mid;
            // std::cout << "Motor settings " << leftMotor << ", " << rightMotor << ", " << result << std::endl;
            kinematics_->motorSpeed(leftMotor, rightMotor, [](bool sucess){});
        }
        return 0.0;
    }

    LidarMode LidarNode::get_sensors_mode() {
        return mode.load();
    }

    void LidarNode::estimate_descrete_lidar_pose(double l_norm, double r_norm) {
        double result = l_norm - r_norm;
        // std::cout << "result: " <<result  << std::endl;
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