// pid.cpp
// BPC-PRP project 2025
// xvarec06 & xruzic56
//
// Source file for PID regulator algorithm.


#include "pid.hpp"

namespace algorithms {

    Pid::Pid(double kp, double ki, double kd)
        : kp_(kp), ki_(ki), kd_(kd), prev_error_(0), integral_(0) {}

    double Pid::step(double error, double dt) {
        integral_ += error * dt;
        double derivative = (error - prev_error_) / dt;
        double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
        prev_error_ = error;
        // std::cout << "error: " << error << " derivation: " << derivative << " integral: " << integral_ << std::endl;
        return output;
    }

    void Pid::reset() {
        prev_error_ = 0;
        integral_ = 0;
    }

}
