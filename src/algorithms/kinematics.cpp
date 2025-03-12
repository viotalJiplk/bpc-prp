#include "kinematics.hpp"//
// Created by student on 12.3.25.
//
namespace algorithms {
    Kinematics::Kinematics(double wheel_radius, double wheel_base, int ticks_revolution) {
        wheel_radius_ = wheel_radius;
        wheel_base_ = wheel_base;
        ticks_revolution_ = ticks_revolution;
    }
    RobotSpeed Kinematics::forward(WheelSpeed x) const {
        RobotSpeed r;
        r.v = (x.l + x.r)/2;
        r.w = (x.r - x.l)/wheel_base_;
        return r;
    }
    WheelSpeed Kinematics::inverse(RobotSpeed x) const {
        WheelSpeed ws;
        ws.l = 0;
        ws.r = 0;
        return ws;
    }
    Coordinates Kinematics::forward(Encoders x) const {
        Coordinates c;
        c.x = 0;
        c.y = 0;
        return c;
    }
    Encoders Kinematics::inverse(Coordinates x) const {
        Encoders e;
        e.l = 0;
        e.r = 0;
        return e;
    }
}
