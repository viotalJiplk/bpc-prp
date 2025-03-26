#include "kinematics_node.hpp"
#include "helper.hpp"

#define WHEEL_BASE 128.0
#define WHEEL_RADIUS 33.0
#define PULSES_PER_ROTATION 576

std::mutex planMutex;

bool hasFinished(uint32_t start, uint32_t actual,  int expectedChange) {
    // TODO catch overflow and underflow
    if (expectedChange > 0) {
        if (UINT32_MAX - start < expectedChange) {
            return  actual > (start + expectedChange);
        } else if ( actual > (UINT32_MAX/2)) {
            return false;
        }else {
            uint32_t expectedWrapped = expectedChange - (UINT32_MAX - start);
            return actual > expectedWrapped;
        }
    } else {
        if (abs(expectedChange) < start) {
            return actual < (start + expectedChange);
        } else if (actual < (UINT32_MAX/2)) {
            return false;
        } else {
            uint32_t expectedWrapped = UINT32_MAX - (expectedChange + start);
            return actual < expectedWrapped;
        }
    }
}

namespace nodes {
    KinematicsNode::KinematicsNode(std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor): GeneralNode("Kinematics", 2) {
        motors_ = std::make_shared<nodes::Motors>();
        executor->add_node(motors_);
        encoders_ = std::make_shared<nodes::Encoder>();
        executor->add_node(encoders_);
        algo_ = new algorithms::Kinematics(WHEEL_RADIUS, WHEEL_BASE, PULSES_PER_ROTATION);
        plan_.hasFinished = true;
        encodersSubscriber_ = this->create_subscription<std_msgs::msg::UInt32MultiArray>(
                Topic::encoders_publisher, 1, std::bind(&KinematicsNode::on_encoder_callback, this, std::placeholders::_1));
    }

    void KinematicsNode::on_encoder_callback(std_msgs::msg::UInt32MultiArray_<std::allocator<void>>::SharedPtr msg) {
        // TODO add evaluation of encoders acording to current command and make corrections
        uint32_t leftMotor = msg->data[0];
        uint32_t rightMotor = msg->data[1];
        planMutex.lock();
        bool localFinished = false;
        std::function<void(bool)> localCallback = [&](bool) {};
        if ((!plan_.hasFinished)) {
            localCallback = plan_.callback;
            bool leftFinished = hasFinished(plan_.start.l, leftMotor, plan_.change.l);
            bool rightFinished = hasFinished(plan_.start.r, rightMotor, plan_.change.r);
            if (leftFinished || rightFinished) {
                localFinished = true;
                motors_->setMotorsSpeed(0,0);
                plan_.hasFinished = true;
                plan_.lMotor = 0;
                plan_.rMotor = 0;
            }else {
                motors_->setMotorsSpeed(plan_.lMotor, plan_.rMotor);
            }
        }else {
            motors_->setMotorsSpeed(0, 0);
        }
        planMutex.unlock();
        if (localFinished) {
            localCallback(true);
        }
    }
    void KinematicsNode::forward(uint32_t length, int16_t speed, std::function<void(bool)> callback){
        algorithms::Coordinates coordinates;
        coordinates.x = length;
        coordinates.y = 0;
        planMutex.lock();
        plan_.start.l = encoders_->getLeftEncoderState();
        plan_.start.r = encoders_->getRightEncoderState();
        plan_.change = algo_->inverse(coordinates);
        plan_.lMotor = speed;
        plan_.rMotor = speed;
        std::function<void(bool)> localCallback = plan_.callback;
        bool localFinished = plan_.hasFinished;
        plan_.hasFinished = false;
        plan_.callback = callback;
        planMutex.unlock();
        if (!localFinished) {
            localCallback(false);
        }
    }

    void KinematicsNode::angle(double angle, int16_t speed, std::function<void(bool)> callback){
        // TODO implement
        algorithms::RobotSpeed calculatedSpeed;
        calculatedSpeed.v = 0;
        calculatedSpeed.w = angle;
        algorithms::WheelAngularSpeed ws = algo_->inverse(calculatedSpeed);
        planMutex.lock();
        plan_.start.l = encoders_->getLeftEncoderState();
        plan_.start.r = encoders_->getRightEncoderState();
        plan_.change = algo_->convertEnc(ws);
        if (angle > 0) {
            plan_.lMotor = -speed;
            plan_.rMotor = speed;
        }else {
            plan_.lMotor = speed;
            plan_.rMotor = -speed;
        }
        std::function<void(bool)> localCallback = plan_.callback;
        bool localFinished = plan_.hasFinished;
        plan_.hasFinished = false;
        plan_.callback = callback;
        planMutex.unlock();
        if (!localFinished) {
            localCallback(false);
        }
    }
}