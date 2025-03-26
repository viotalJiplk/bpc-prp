#ifndef PID_HPP
#define PID_HPP

#pragma once

#include <iostream>
#include <chrono>

namespace algorithms {

    class Pid {
    public:
        Pid(float kp, float ki, float kd);
        float step(float error, float dt);
        void reset();

    private:
        float kp_;
        float ki_;
        float kd_;
        float prev_error_;
        float integral_;
    };
}

#endif // PID_HPP
