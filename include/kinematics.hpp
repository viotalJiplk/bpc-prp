//
// Created by student on 12.3.25.
//

#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP
namespace algorithms {
    struct RobotSpeed{
        float v; //linear
        float w; //angluar
    };

    struct WheelAngularSpeed{ //depends on you in what units
        float l; //left
        float r; //right
    };

    struct Encoders{
        int l; //left
        int r; //right
    };
    struct Coordinates{ //Cartesian coordinates
        float x;
        float y;
    };

    class Kinematics{
    public:
        Kinematics(double wheel_radius, double wheel_base, int ticks_revolution);
        RobotSpeed forward(WheelAngularSpeed x) const;
        WheelAngularSpeed inverse(RobotSpeed x) const;
        Coordinates forward(Encoders x) const;
        Encoders inverse(Coordinates x) const;
    private:
        double wheel_radius_;
        double wheel_base_;
        double wheel_circumference;
        int ticks_revolution_;
    };
}
#endif //KINEMATICS_HPP
