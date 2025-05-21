#include "kinematics_node.hpp"
#include "helper.hpp"

#define WHEEL_BASE 128.0
#define WHEEL_RADIUS 35.0
#define PULSES_PER_ROTATION 576

std::mutex planMutex;

bool hasFinished(uint32_t start, uint32_t actual,  int expectedChange) {
    // TODO catch overflow and underflow
    bool toReturn = false;
    if (expectedChange > 0) {
        if (UINT32_MAX - start > expectedChange) {
            toReturn =  actual > (start + expectedChange);
        } else if ( actual > (UINT32_MAX/2)) {
            toReturn = false;
        }else {
            uint32_t expectedWrapped = expectedChange - (UINT32_MAX - start);
            return actual > expectedWrapped;
        }
    } else {
        if (abs(expectedChange) < start) {
            toReturn = actual < (start + expectedChange);
        } else if (actual < (UINT32_MAX/2)) {
            toReturn = false;
        } else {
            uint32_t expectedWrapped = UINT32_MAX - (expectedChange + start);
            toReturn = actual < expectedWrapped;
        }
    }
    if (toReturn)
    {
        return true;
    }else
    {
        return false;
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
                Topic::encoders_publisher, 1, std::bind(&KinematicsNode::on_encoder_callback, this, std::placeholders::_1));\
        count.store(0);
    }

    void KinematicsNode::on_encoder_callback(std_msgs::msg::UInt32MultiArray_<std::allocator<void>>::SharedPtr msg)
    {
        // TODO add evaluation of encoders acording to current command and make corrections
        uint32_t leftMotor = msg->data[0];
        uint32_t rightMotor = msg->data[1];
        this->lEncoder.store(leftMotor);
        this->rEncoder.store(rightMotor);
        uint8_t tmpCount = count.exchange(count.load()+1);
        if (tmpCount > 3){
            count.store(0);
            planMutex.lock();
            bool localFinished = false;
            std::function<void(bool)> localCallback = [&](bool) {};
            if (plan_.isInfinite) {
                motors_->setMotorsSpeed(plan_.lMotor, plan_.rMotor);
            }else if ((!plan_.hasFinished)) {
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
        plan_.isInfinite = false;
        planMutex.unlock();
        if (!localFinished) {
            localCallback(false);
        }
    }
    void KinematicsNode::backward(uint32_t length, int16_t speed, std::function<void(bool)> callback) {
        algorithms::Coordinates coordinates;
        coordinates.x = length;
        coordinates.y = 0;
        planMutex.lock();
        plan_.start.l = encoders_->getLeftEncoderState();
        plan_.start.r = encoders_->getRightEncoderState();
        plan_.change = algo_->inverse(coordinates);
        plan_.change.l = -plan_.change.l;
        plan_.change.r = -plan_.change.r;
        plan_.lMotor = -speed;
        plan_.rMotor = -speed;
        std::function<void(bool)> localCallback = plan_.callback;
        bool localFinished = plan_.hasFinished;
        plan_.hasFinished = false;
        plan_.callback = callback;
        plan_.isInfinite = false;
        planMutex.unlock();
        if (!localFinished) {
            localCallback(false);
        }
    }

    void KinematicsNode::interruptOp()
    {
        planMutex.lock();
        struct planPaused  plan_paused;
        plan_paused.lEncoder = this->lEncoder;
        plan_paused.rEncoder = this->rEncoder;
        plan_paused.plan = plan_;
        planStack.push(plan_paused);
        motors_->setMotorsSpeed(0,0);
        plan_.hasFinished = true;
        plan_.lMotor = 0;
        plan_.rMotor = 0;
        planMutex.unlock();
    }

    void KinematicsNode::continueOp(){
        planMutex.lock();
        if (!planStack.empty())
        {
            struct planPaused plan_paused = planStack.top();
            planStack.pop();
            plan_ = plan_paused.plan;
            plan_.change.l += this->lEncoder - plan_paused.lEncoder;
            plan_.change.r += this->rEncoder - plan_paused.rEncoder;
        }else
        {
            std::cout << "Cannot continue with nonexisting plan" << std::endl;
        }
        planMutex.unlock();
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
        plan_.isInfinite = false;
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

    void KinematicsNode::motorSpeed(int16_t speedL, int16_t speedR, std::function<void(bool)> callback) {
        motorSpeed(speedL, speedR, false, callback);
    }

    void KinematicsNode::motorSpeed(int16_t speedL, int16_t speedR, bool isInfinite, std::function<void(bool)> callback){
        // TODO implement
        planMutex.lock();
        plan_.start.l = encoders_->getLeftEncoderState();
        plan_.start.r = encoders_->getRightEncoderState();
        plan_.change.l = 200;
        plan_.change.r = 200;
        plan_.lMotor = speedL;
        plan_.rMotor = speedR;
        plan_.isInfinite = isInfinite;
        std::function<void(bool)> localCallback = plan_.callback;
        bool localFinished = plan_.hasFinished;
        plan_.hasFinished = false;
        plan_.callback = callback;
        planMutex.unlock();
        if (!localFinished) {
            localCallback(false);
        }
    }

    void KinematicsNode::stop(std::function<void(bool)> callback) {
        planMutex.lock();
        plan_.start.l = encoders_->getLeftEncoderState();
        plan_.start.r = encoders_->getRightEncoderState();
        plan_.change.l = 0;
        plan_.change.r = 0;
        plan_.lMotor = 0;
        plan_.rMotor = 0;
        plan_.isInfinite = false;
        std::function<void(bool)> localCallback = plan_.callback;
        bool localFinished = plan_.hasFinished;
        plan_.hasFinished = false;
        plan_.callback = callback;
        while (!this->planStack.empty())
        {
            this->planStack.pop();
        }
        planMutex.unlock();
        if (!localFinished) {
            localCallback(false);
        }
    }
    void KinematicsNode::turnLeft(int16_t speed, std::function<void(bool)> callback){
        this->angle(M_PI/2.0, speed, [callback](bool sucess) {
                        callback(sucess);
        });
    }
    void KinematicsNode::turnRight(int16_t speed, std::function<void(bool)> callback){
        this->angle(-M_PI/2.0, speed, [callback](bool sucess) {
                        callback(sucess);
        });
    }
    void KinematicsNode::turnBack(int16_t speed, std::function<void(bool)> callback){
        this->angle(M_PI, speed, [callback](bool sucess) {
                        callback(sucess);
        });
    }

    void KinematicsNode::turnSlightlyLeft(int16_t speed, std::function<void(bool)> callback){
        this->angle(M_PI/4.0, speed, [callback](bool sucess) {
                        callback(sucess);
        });
    }
    void KinematicsNode::turnSlightlyRight(int16_t speed, std::function<void(bool)> callback){
        this->angle(-M_PI/4.0, speed, [callback](bool sucess) {
                        callback(sucess);
        });
    }
}