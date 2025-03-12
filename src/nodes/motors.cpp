#include "motors.hpp"
#include "helper.hpp"

namespace nodes {
    Motors::Motors(): Node("MotorsPublisher") {
            motors_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(Topic::set_motor_speeds, 1);
    }

    void Motors::setMotorsSpeed(uint8_t left, uint8_t right){
        auto motors = std_msgs::msg::UInt8MultiArray();
        motors.data = {left, right};
        motors_publisher_->publish(motors);
    }
}
