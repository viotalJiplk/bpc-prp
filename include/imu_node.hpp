
#ifndef IMU_NODE_HPP
#define IMU_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "imu.hpp"
#include "pid.hpp"
#include "kinematics_node.hpp"


namespace nodes {

    enum class ImuNodeMode {
        CALIBRATE,
        INTEGRATE,
        None,
    };

    class ImuNode : public rclcpp::Node {
    public:
        ImuNode(std::shared_ptr<KinematicsNode> kinematics);
        ~ImuNode() override = default;

        // Set the IMU Mode
        void setMode(const ImuNodeMode setMode);

        // Get the current IMU Mode
        ImuNodeMode getMode();

        // Get the results after Integration
        auto getIntegratedResults();

        // Reset the class
        void reset_imu();
        void start();
        void stop();

    private:

        void calibrate();
        void integrate(float gyro_z, double dt);

        long getTimestamp();

        std::atomic<long> prevT_;

        ImuNodeMode mode = ImuNodeMode::INTEGRATE;

        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
        algorithms::PlanarImuIntegrator planar_integrator_;

        std::vector<float> gyro_calibration_samples_;
        int sample_num;

        std::shared_ptr<KinematicsNode> kinematics_;

        algorithms::Pid* algo_;

        void on_imu_msg(const sensor_msgs::msg::Imu::SharedPtr msg);
    };
}

#endif //IMU_NODE_HPP
