// imu.cpp
// BPC-PRP project 2025
// xvarec06 & xruzic56
//
// Source file for IMU data interpretation node.


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "imu.hpp"
#include "imu_node.hpp"
#include <chrono>
#include "helper.hpp"
#include "pid.hpp"


struct pid_imu {
    double kp;
    double ki;
    double kd;
    double mid;
    double deviation;
};

struct pid_imu pidImuValues = {
    .kp = 5.0,
    .ki = 0.01,
    .kd = 0.1,
    .mid = 10.0,
    .deviation = 20.0,
};

namespace nodes {

    ImuNode::ImuNode(std::shared_ptr<KinematicsNode> kinematics): rclcpp::Node("ImuNode") {
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
          Topic::imu, 1, std::bind(&ImuNode::on_imu_msg, this, std::placeholders::_1));
        kinematics_ = kinematics;
        mode = ImuNodeMode::None;
        planar_integrator_ = algorithms::PlanarImuIntegrator();
        gyro_calibration_samples_ = std::vector<float>();
        sample_num = 0;
        prevT_.store(0);
        algo_ = new algorithms::Pid(pidImuValues.kp, pidImuValues.ki, pidImuValues.kd);
    }

    void ImuNode::setMode(const ImuNodeMode setMode) {
      mode = setMode;
    }

    auto ImuNode::getIntegratedResults() {

    }
    void ImuNode::start() {
        reset_imu();
        mode = ImuNodeMode::CALIBRATE;
    }

    void ImuNode::stop() {
        mode = ImuNodeMode::None;
    }

    ImuNodeMode ImuNode::getMode() {
        return mode;
    }

    void ImuNode::reset_imu() {
        mode = ImuNodeMode::CALIBRATE;
        gyro_calibration_samples_ = std::vector<float>();
        sample_num = 0;
        prevT_.store(0);
    }

    void ImuNode::calibrate() {
        planar_integrator_.setCalibration(gyro_calibration_samples_);
    }

    void ImuNode::integrate(float gyro_z, double dt) {

    }

    long ImuNode::getTimestamp() {
        auto timeStamp = std::chrono::high_resolution_clock::now();
        long timeNow = std::chrono::duration_cast<std::chrono::milliseconds>(
            timeStamp.time_since_epoch()
        ).count();
        return timeNow;
    }

    void ImuNode::on_imu_msg(const sensor_msgs::msg::Imu::SharedPtr msg) {
        if(getMode() == ImuNodeMode::CALIBRATE) {
          gyro_calibration_samples_.push_back(msg->angular_velocity.z);
          sample_num++;
          if(sample_num >= 240) {
            calibrate();
            setMode(ImuNodeMode::INTEGRATE);
          }
        }
        else if(getMode() == ImuNodeMode::INTEGRATE) {
            long timeNow = getTimestamp();
            long oldTime = prevT_.exchange(timeNow);
            if (oldTime != 0) {
                double dt = (timeNow-oldTime)/1000.0;
                float e = planar_integrator_.update(msg->angular_velocity.z, dt);
                float diff = algo_->step(e, 1);
                if (diff > 1) {
                    diff = 1.0;
                }else if (diff < -1) {
                    diff = -1.0;
                }
                if (abs(e) > 0.01) {
                    int leftMotor = (diff)*pidImuValues.deviation+pidImuValues.mid;
                    int rightMotor = (-diff)*pidImuValues.deviation+pidImuValues.mid;
                    std::cout << diff << std::endl;
                    kinematics_->motorSpeed(leftMotor, rightMotor, [](bool sucess){});
                }else {
                    kinematics_->motorSpeed(10, 10, [](bool sucess){});
                }
            }
        }
    }

}


