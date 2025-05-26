// imu.cpp
// BPC-PRP project 2025
// xvarec06 & xruzic56
//
// Source file for IMU data processing algorithm.


#include "imu.hpp"
#include <cmath>


namespace algorithms {

    PlanarImuIntegrator::PlanarImuIntegrator(){
        theta_ = 0.0f;
        gyro_offset_ = 0.0f;
    };

    PlanarImuIntegrator::~PlanarImuIntegrator(){};

    float PlanarImuIntegrator::vectorMean(std::vector<float> vector){
        float sum = 0.0f;
        for (float num : vector) {
            sum += num;
        }
        return sum/vector.size();
    }

    float PlanarImuIntegrator::update(float gyro_z, double dt) {
        float old_theta = theta_;
        theta_ += dt * (gyro_z - gyro_offset_);
        return old_theta - theta_; // return difference from last measurement
    }

    void PlanarImuIntegrator::setCalibration(std::vector<float> gyro) {
        gyro_offset_ = vectorMean(gyro);
    }

    float PlanarImuIntegrator::getYaw() {
      return theta_;
    }

    void PlanarImuIntegrator::resetYaw() {
      theta_ = 0.0;
    }

    void PlanarImuIntegrator::reset() {
      theta_ = 0.0f;
      gyro_offset_ = 0.0f;
    }


}
