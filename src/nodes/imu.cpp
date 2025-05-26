// imu.cpp
// BPC-PRP project 2025
// xvarec06 & xruzic56
//
// Source file for IMU data interpretation node.


#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "imu.hpp"
#include "imu_node.hpp"
#include <chrono>
#include "helper.hpp"
#include "io_node.hpp"
#include "motors.hpp"
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

    ImuNode::ImuNode(std::shared_ptr<KinematicsNode> kinematics, std::shared_ptr<Motors> motors, std::shared_ptr<IoNode> ionode): rclcpp::Node("ImuNode") {
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
          Topic::imu, 1, std::bind(&ImuNode::on_imu_msg, this, std::placeholders::_1));
        kinematics_ = kinematics;
        motors_ = motors;
        ionode_ = ionode;
        mode = ImuNodeMode::NONE;
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
        motors_->setMotorsSpeed(0,0);
        mode = ImuNodeMode::NONE;
    }

    ImuNodeMode ImuNode::getMode() {
        return mode;
    }

    void ImuNode::turnLeft() {
        mode = ImuNodeMode::LEFT90;
    }

    void ImuNode::turnRight() {
        mode = ImuNodeMode::RIGHT90;
    }

    void ImuNode::turnBack() {
        mode = ImuNodeMode::RIGHT180;
    }
    
    void ImuNode::forward() {   // TODO distance?
        mode = ImuNodeMode::FORWARD;
    }

    void ImuNode::reset_imu() {
        mode = ImuNodeMode::CALIBRATE;
        gyro_calibration_samples_ = std::vector<float>();
        sample_num = 0;
        prevT_.store(0);
        algo_->reset();
    }

    // estimated yaw and time records resets, calibration stays
    void ImuNode::reset_orientation() {
        planar_integrator_.resetYaw();
        prevT_.store(0);
    }

    // pass samples recorded while staying still to yaw estimation algorithm
    void ImuNode::calibrate() {
        planar_integrator_.setCalibration(gyro_calibration_samples_);
    }

    long ImuNode::getTimestamp() {
        auto timeStamp = std::chrono::high_resolution_clock::now();
        long timeNow = std::chrono::duration_cast<std::chrono::milliseconds>(
            timeStamp.time_since_epoch()
        ).count();
        return timeNow;
    }

    // do integration step with new data to update estimated yaw (return timestamp of the previous measurement - for PID later)
    long ImuNode::integrate(float angular_velocity_z) {
        long timeNow = getTimestamp();
        long oldTime = prevT_.exchange(timeNow);
        float difference = 0.0;
        if (oldTime != 0) {
            double dt = (timeNow-oldTime)/1000.0;
            difference = planar_integrator_.update(angular_velocity_z, dt);   
        }
        return oldTime;
    }

    // PID control of motors by IMU (returns angles difference)
    double ImuNode::control_movement(float desired_angle, float current_angle, long current_timestamp, long previous_timestamp) {
        algo_->reset(); // reset PID controller
        
        // desired angle minus current angle - "error" and time difference between last 2 measurements - "dt"
        double angle_difference = desired_angle - current_angle;
        double timestamp_difference = current_timestamp - previous_timestamp;

        // PID step
        double correction = algo_->step(angle_difference, timestamp_difference);

        // round big results to <-1,1> interval - result is coefficient for motors
        if(correction > 1) { correction = 1.0; }
        else if(correction < -1) { correction = -1.0; }

        // set motors speed to execute correction
        int16_t leftMotor = (correction) * pidImuValues.deviation + pidImuValues.mid;
        int16_t rightMotor = (-correction) * pidImuValues.deviation + pidImuValues.mid;
        motors_->setMotorsSpeed(leftMotor, rightMotor);

        return angle_difference;
    }

    // callback on every imu message
    void ImuNode::on_imu_msg(const sensor_msgs::msg::Imu::SharedPtr msg) {

        long prev_timestamp = 0;
        
        long timeNow, oldTime;

        switch(this->mode) {
            case ImuNodeMode::CALIBRATE:
                gyro_calibration_samples_.push_back(msg->angular_velocity.z);
                sample_num++;
                if(sample_num >= 420) {
                    calibrate(); // pass samples to algorithm
                    this->mode = ImuNodeMode::NONE;
                    ionode_->led_blink(0, 200);
                }
                break;

            case ImuNodeMode::INTEGRATE:
                // original implementation of forward movement - not used in maze node
                timeNow = getTimestamp();
                oldTime = prevT_.exchange(timeNow);
                if (oldTime != 0) {
                    double dt = (timeNow-oldTime)/1000.0;
                    float e = planar_integrator_.update(msg->angular_velocity.z, dt);
                    float diff = algo_->step(e, 1);
                    if (diff > 1) {
                        diff = 1.0;
                    }
                    else if (diff < -1) {
                        diff = -1.0;
                    }
            
                    if (abs(e) > 0.01) {
                        int leftMotor = (diff)*pidImuValues.deviation+pidImuValues.mid;
                        int rightMotor = (-diff)*pidImuValues.deviation+pidImuValues.mid;
                        std::cout << diff << std::endl;
                        kinematics_->motorSpeed(leftMotor, rightMotor, [](bool sucess){});
                    }
                    else {
                        kinematics_->motorSpeed(10, 10, [](bool sucess){});
                    }
                }
                break;

            case ImuNodeMode::LEFT90:
                // process data from IMU
                prev_timestamp = integrate(msg->angular_velocity.z);

                // stop if no further rotation needed
                if( control_movement(-90.0f, planar_integrator_.getYaw(), this->prevT_, prev_timestamp) < 0.01 ) {
                    // difference too small, job done --> return to mode "none"
                    kinematics_->stop();
                    //motors_->setMotorsSpeed(0, 0);
                    this->reset_orientation();
                    this->mode = ImuNodeMode::NONE;   
                }
                // else continue in this mode
                break;

            case ImuNodeMode::RIGHT90:
                // process data from IMU
                prev_timestamp = integrate(msg->angular_velocity.z);

                // stop if no further rotation needed
                if( control_movement(90.0f, planar_integrator_.getYaw(), this->prevT_, prev_timestamp) < 0.01 ) {
                    // difference too small, job done --> return to mode "none"
                    kinematics_->stop();
                    //motors_->setMotorsSpeed(0, 0);
                    this->reset_orientation();
                    this->mode = ImuNodeMode::NONE;   
                }
                // else continue in this mode
                break;

            case nodes::ImuNodeMode::RIGHT180:
                // process data from IMU
                prev_timestamp = integrate(msg->angular_velocity.z);

                // stop if no further rotation needed
                if( control_movement(180.0f, planar_integrator_.getYaw(), this->prevT_, prev_timestamp) < 0.01 ) {
                    // difference too small, job done --> return to mode "none"
                    kinematics_->stop();
                    //motors_->setMotorsSpeed(0, 0);
                    this->reset_orientation();
                    this->mode = ImuNodeMode::NONE;   
                }
                // else continue in this mode
                break;

            case ImuNodeMode::FORWARD:

                // TODO decide: use IMU or kinematics? 
                
                // temporarily using kinematics to move forward - fixed distance
                this->kinematics_->forward(50, 10, [](bool sucess){});

                /*
                // process data from IMU
                prev_timestamp = integrate(msg->angular_velocity.z);

                control_movement(0.0f, planar_integrator_.getYaw(), this->prevT_, prev_timestamp);

                // TODO come up with stopping condition
                if( ) {
                    motors_->setMotorsSpeed(0, 0);
                    this->reset_orientation();
                    this->mode = ImuNodeMode::NONE;   
                }
                */

                break;

            case ImuNodeMode::NONE:
            default:
                // do nothing, wait for mode change
                break;
        }

    }

}


