// imu_node.hpp
// BPC-PRP project 2025
// xvarec06 & xruzic56
//
// Header file for IMU data interpretation node.


#ifndef IMU_NODE_HPP
#define IMU_NODE_HPP

#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "imu.hpp"
#include "io_node.hpp"
#include "pid.hpp"
#include "kinematics_node.hpp"


namespace nodes {

    enum class ImuNodeMode {
        CALIBRATE,
        INTEGRATE,
        NONE,
        LEFT90,
        RIGHT90,
        RIGHT180,
        FORWARD,
    };

    class ImuNode : public rclcpp::Node {
    public:
        ImuNode(std::shared_ptr<KinematicsNode> kinematics, std::shared_ptr<Motors> motors, std::shared_ptr<IoNode> ionode);
        ~ImuNode() override = default;

        // Set the IMU Mode
        void setMode(const ImuNodeMode setMode);

        // Get the current IMU Mode
        ImuNodeMode getMode();

        // Get the results after Integration
        auto getIntegratedResults();

        // Turns etc. for maze solving
        void turnLeft();
        void turnRight();
        void turnBack();
        void forward(); // TODO distance?
        void reset_orientation();

        // Reset the class
        void reset_imu();
        void start();
        void stop();

    private:

        void calibrate();
        long integrate(float angular_velocity_z); // returns timestamp of previous measurement

        double control_movement(float desired_angle, float current_angle, long current_timestamp, long previous_timestamp); // PID wrapper, returns angles difference

        long getTimestamp();

        std::atomic<long> prevT_;

        ImuNodeMode mode = ImuNodeMode::INTEGRATE;

        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
        algorithms::PlanarImuIntegrator planar_integrator_;

        std::vector<float> gyro_calibration_samples_;
        int sample_num;

        std::shared_ptr<KinematicsNode> kinematics_;
        std::shared_ptr<Motors> motors_;
        std::shared_ptr<IoNode> ionode_;

        algorithms::Pid* algo_;

        void on_imu_msg(const sensor_msgs::msg::Imu::SharedPtr msg);
    };
}

#endif //IMU_NODE_HPP
