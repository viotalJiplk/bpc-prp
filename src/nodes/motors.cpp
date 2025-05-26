// motors.cpp
// BPC-PRP project 2025
// xvarec06 & xruzic56
//
// Source file for motors driving node (used by kinematics).


#include "motors.hpp"
#include "helper.hpp"

uint8_t convert(int16_t number) {
    if (number > (UINT8_MAX - 127)) {
        return UINT8_MAX;
    }else if ((number + 127) < 0 ) {
        return 0;
    }else {
        return (uint8_t) (number + 127);
    }
}

namespace nodes {
    Motors::Motors(): Node("MotorsPublisher") {
            motors_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(Topic::set_motor_speeds, 1);
    }

    void Motors::setMotorsSpeed(int16_t left, int16_t right){
        auto motors = std_msgs::msg::UInt8MultiArray();
        motors.data = {convert(left), convert(right)};
        motors_publisher_->publish(motors);
    }
}
