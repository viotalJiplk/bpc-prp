// kinematics.cpp
// BPC-PRP project 2025
// xvarec06 & xruzic56
//
// Source file for differential drive kinematics algorithm.


#include "kinematics.hpp"

#include <cmath>
#define PI 3.14159265358979323846

namespace algorithms {
    Kinematics::Kinematics(double wheel_radius, double wheel_base, int ticks_revolution) {
        wheel_radius_ = wheel_radius;
        wheel_circumference = wheel_radius*2*PI;
        wheel_base_ = wheel_base;
        ticks_revolution_ = ticks_revolution;
    }

    /**
     * Calculates robot speed form wheel speed
     * @param x
     * @return
     */
    RobotSpeed Kinematics::forward(WheelAngularSpeed x) const {
        RobotSpeed r;
        r.v = (x.l*wheel_radius_ + x.r*wheel_radius_)/2;
        r.w = (x.r*wheel_radius_ - x.l*wheel_radius_)/wheel_base_;
        return r;
    }

    /**
     * Calculates wheel speed from robot speed
     * @param x
     * @return
     */
    WheelAngularSpeed Kinematics::inverse(RobotSpeed x) const {
        WheelAngularSpeed ws;
        ws.l = (x.v-(x.w*wheel_base_)/2)/wheel_radius_;
        ws.r = (x.v+(x.w*wheel_base_)/2)/wheel_radius_;
        return ws;
    }

    /**
     * Calculates expected coordinates change from encoders change
     * @param enc
     * @return
     */
    Coordinates Kinematics::forward(EncodersChange enc) const {
        WheelAngularSpeed ws = convertEnc(enc);
        RobotSpeed rs = forward(ws);
        Coordinates c;
        c.x = rs.v*cos(rs.w);
        c.y = rs.v*sin(rs.w);
        return c;
    }

    /**
     * Calculates expected coordinates change from encoders change
     * @param enc
     * @return
     */
    CoordinatesWithDirectionChange Kinematics::forwardDirection(EncodersChange enc) const {
        WheelAngularSpeed ws = convertEnc(enc);
        RobotSpeed rs = forward(ws);
        CoordinatesWithDirectionChange c;
        c.x = rs.v*cos(rs.w);
        c.y = rs.v*sin(rs.w);
        c.theta = rs.w; // this could be done because t = 1
        return c;
    }

    /**
     * Calculates expected encoders change from coordinates change
     * @param coords
     * @return
     */
    EncodersChange Kinematics::inverse(Coordinates coords) const {
        RobotSpeed rs;
        rs.v = sqrt(coords.x*coords.x + coords.y*coords.y);
        rs.w = atan2(coords.y, coords.x);
        WheelAngularSpeed ws = inverse(rs);
        return convertEnc(ws);
    }

    /**
     * Calculates expected encoders change from coordinates change
     * @param coords
     * @return
     */
    EncodersWithDirectionChange Kinematics::inverseDirection(Coordinates coords) const {
        RobotSpeed rs;
        rs.v = sqrt(coords.x*coords.x + coords.y*coords.y);
        rs.w = atan2(coords.y, coords.x);
        WheelAngularSpeed ws = inverse(rs);
        EncodersChange enc = convertEnc(ws);
        EncodersWithDirectionChange encChange ={enc.l, enc.r, rs.w};
        return encChange;
    }

    /**
     * Calculates wheel angle (rads) change from encoder number
     * @param enc
     * @return
     */
    WheelAngularSpeed Kinematics::convertEnc(EncodersChange enc) const { // this is not really a speed (there is no time)
        WheelAngularSpeed ws;
        ws.l = static_cast<double>(enc.l)/static_cast<double>(ticks_revolution_)*2.0*PI;
        ws.r = static_cast<double>(enc.r)/static_cast<double>(ticks_revolution_)*2.0*PI;
        return ws;
    };

    /**
     * Calculates encoder values from wheel angle (rads) change
     * @param x
     * @return
     */
    EncodersChange Kinematics::convertEnc(WheelAngularSpeed x) const { //
        EncodersChange enc;
        enc.l = static_cast<int>((x.l/(2.0*PI))*static_cast<double>(ticks_revolution_));
        enc.r = static_cast<int>((x.r/(2.0*PI))*static_cast<double>(ticks_revolution_));
        return enc;
    };
}
