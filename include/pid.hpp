// pid.hpp
// BPC-PRP project 2025
// xvarec06 & xruzic56
//
// Header file for PID regulator algorithm.


#ifndef PID_HPP
#define PID_HPP

#pragma once

#include <iostream>
#include <chrono>

namespace algorithms {

    class Pid {
    public:
        Pid(double kp, double ki, double kd);
        double step(double error, double dt);
        void reset();

    private:
        double kp_;
        double ki_;
        double kd_;
        double prev_error_;
        double integral_;
    };
}

#endif // PID_HPP
