
#ifndef IMU_HPP
#define IMU_HPP

#include <iostream>
#include <cmath>
#include <numeric>
#include <vector>

namespace algorithms {

    class PlanarImuIntegrator {
    public:

        PlanarImuIntegrator();
        ~PlanarImuIntegrator();

        float vectorMean(std::vector<float> vector);

        // TODO: Call this regularly to integrate gyro_z over time
        float update(float gyro_z, double dt);

        // TODO: Calibrate the gyroscope by computing average from static samples
        void setCalibration(std::vector<float> gyro);

        // TODO: Return the current estimated yaw
        float getYaw();

        // TODO: Reset orientation and calibration
        void reset();

    private:
        float firstTheta_;
        float theta_;       // Integrated yaw angle (radians)
        float gyro_offset_; // Estimated gyro bias
    };
}

#endif //IMU_HPP
