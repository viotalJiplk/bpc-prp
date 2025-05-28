# CMakeLists.txt
# BPC-PRP project 2025
# xvarec06 & xruzic56
#
# CMakeLists for tests build.


cmake_minimum_required(VERSION 3.20)

include_directories(../include)

find_package(GTest REQUIRED)
enable_testing()

macro(make_units_test name)
    add_executable(${name} t_${name}.cpp ${SRC})
    target_link_libraries(${name} GTest::GTest GTest::Main)
    ament_target_dependencies(${name} rclcpp std_msgs sensor_msgs cv_bridge image_transport visualization_msgs)
    add_test(NAME ${name} COMMAND ${name})
endmacro()

# make_units_test(example)
# make_units_test(kinematics)
# make_units_test(lineEstimator)
