#include "kinematics.hpp"//
#define PI 3.14159265358979323846
// Created by student on 12.3.25.
//
namespace algorithms {
    Kinematics::Kinematics(double wheel_radius, double wheel_base, int ticks_revolution) {
        wheel_radius_ = wheel_radius;
        wheel_circumference = wheel_radius*2*PI;
        wheel_base_ = wheel_base;
        ticks_revolution_ = ticks_revolution;
    }
    RobotSpeed Kinematics::forward(WheelAngularSpeed x) const {
        RobotSpeed r;
        r.v = (x.l + x.r)/2;
        r.w = (x.r - x.l)/wheel_base_;
        return r;
    }
    WheelAngularSpeed Kinematics::inverse(RobotSpeed x) const {
        WheelAngularSpeed ws;
        ws.l = (x.v-(x.w*wheel_base_)/2)/wheel_radius_;
        ws.r = (x.v+(x.w*wheel_base_)/2)/wheel_radius_;
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
