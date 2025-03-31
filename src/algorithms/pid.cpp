#include "pid.hpp"

namespace algorithms {

    Pid::Pid(float kp, float ki, float kd)
        : kp_(kp), ki_(ki), kd_(kd), prev_error_(0), integral_(0) {}

    float Pid::step(float error, float dt) {
        integral_ += error * dt;
        float derivative = (error - prev_error_) / dt;
        float output = kp_ * error + ki_ * integral_ + kd_ * derivative;
        prev_error_ = error;
        // std::cout << "error: " << error << "output: " << output << std::endl;
        return output;
    }

    void Pid::reset() {
        prev_error_ = 0;
        integral_ = 0;
    }

}
