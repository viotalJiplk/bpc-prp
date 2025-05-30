// kinematics.hpp
// BPC-PRP project 2025
// xvarec06 & xruzic56
//
// Header file for differential drive kinematics algorithm.


#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP
namespace algorithms {
    struct RobotSpeed{
        double v; //linear
        double w; //angluar
    };

    struct WheelAngularSpeed{ //depends on you in what units
        double l; //left
        double r; //right
    };

    struct WheelTraveledLength{
        double l; //left
        double r; //right
    };

    struct EncodersChange{
        int l; //left
        int r; //right
    };

    struct EncodersWithDirectionChange{
        int l; //left
        int r; //right
        double theta;
    };

    struct Coordinates{ //Cartesian coordinates
        double x;
        double y;
    };

    struct CoordinatesWithDirectionChange{
        double x;
        double y;
        double theta;
    };

    class Kinematics{
    public:
        Kinematics(double wheel_radius, double wheel_base, int ticks_revolution);
        RobotSpeed forward(WheelAngularSpeed x) const;
        WheelAngularSpeed inverse(RobotSpeed x) const;
        Coordinates forward(EncodersChange coords) const;
        EncodersChange inverse(Coordinates enc) const;
        CoordinatesWithDirectionChange forwardDirection(EncodersChange enc) const;
        EncodersWithDirectionChange inverseDirection(Coordinates coords) const;
        WheelAngularSpeed convertEnc(EncodersChange x) const;
        EncodersChange convertEnc(WheelAngularSpeed x) const;
    private:
        double wheel_radius_;
        double wheel_base_;
        double wheel_circumference;
        int ticks_revolution_;
    };
}
#endif //KINEMATICS_HPP
