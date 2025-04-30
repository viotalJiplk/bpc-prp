
#ifndef HELPER_HPP
#define HELPER_HPP

#include <iostream>

static const int MAIN_LOOP_PERIOD_MS = 50;

enum class IntersectionType {
    None,
    LeftT,
    RightT,
    TopT,
    AllFour,
};

namespace Topic {
    const std::string buttons = "/bpc_prp_robot/buttons";
    const std::string set_rgb_leds = "/bpc_prp_robot/rgb_leds";
    const std::string set_motor_speeds = "/bpc_prp_robot/set_motor_speeds";
    const std::string encoders = "/bpc_prp_robot/encoders";
    const std::string ultrasound = "/bpc_prp_robot/ultrasounds";
    const std::string line_sensors = "/bpc_prp_robot/line_sensors";
    const std::string mainNode = "/bpc_prp_internal/mainNode";
    const std::string ionode_buttons = "/bpc_prp_internal/ionode_buttons";
    const std::string encoders_publisher = "/bpc_prp_internal/encoders_publisher";
    const std::string ultrasoundFiltered = "/bpc_prp_internal/ultrasoundsFiltered";
    const std::string keyboardIn = "/bpc_prp_internal/keyboardIn";
    const std::string lidar = "/bpc_prp_robot/lidar";
    const std::string lidarFiltered = "/bpc_prp_internal/lidarFiltered";
    const std::string imu = "/bpc_prp_robot/imu";
    const std::string camera = "/bpc_prp_robot/camera/compressed";
    const std::string aruco = "/bpc_prp_internal/aruco";
};
namespace helper {
    long getTimestamp();
}

namespace Frame {
    const std::string origin = "origin";
    const std::string robot = "robot";
    const std::string lidar = "lidar";
};

#endif
