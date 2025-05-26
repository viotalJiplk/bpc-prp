// imu.hpp
// BPC-PRP project 2025
// xvarec06 & xruzic56
//
// Header file for IMU data processing algorithm.


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

        // Call this regularly to integrate gyro_z over time
        float update(float gyro_z, double dt);

        // Calibrate the gyroscope by computing average from static samples
        void setCalibration(std::vector<float> gyro);

        // Return the current estimated yaw
        float getYaw();

        // Reset orientation
        void resetYaw();

        // Reset orientation and calibration
        void reset();

    private:
        float firstTheta_;
        float theta_;       // Integrated yaw angle (radians)
        float gyro_offset_; // Estimated gyro bias
    };
}

#endif //IMU_HPP
