# CMakeLists.txt
# BPC-PRP project 2025
# xvarec06 & xruzic56
#
# CMakeLists for project build.


cmake_minimum_required(VERSION 3.20)
project(prp_project)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(SDL2 REQUIRED)
find_package(visualization_msgs REQUIRED)

set(SRC
        ${CMAKE_SOURCE_DIR}/src/nodes/io_node.cpp
        ${CMAKE_SOURCE_DIR}/src/nodes/encoders.cpp
        ${CMAKE_SOURCE_DIR}/src/nodes/motors.cpp
        ${CMAKE_SOURCE_DIR}/src/algorithms/kinematics.cpp
        ${CMAKE_SOURCE_DIR}/src/algorithms/pid.cpp
        ${CMAKE_SOURCE_DIR}/src/nodes/line.cpp
        ${CMAKE_SOURCE_DIR}/src/nodes/ultrasound.cpp
        ${CMAKE_SOURCE_DIR}/src/nodes/main_node.cpp
        ${CMAKE_SOURCE_DIR}/src/nodes/generalNode.cpp
        ${CMAKE_SOURCE_DIR}/src/nodes/kinematics_node.cpp
        ${CMAKE_SOURCE_DIR}/src/nodes/ultrasound_sensor.cpp
        ${CMAKE_SOURCE_DIR}/src/nodes/keyboard.cpp
        ${CMAKE_SOURCE_DIR}/src/algorithms/lidar.cpp
        ${CMAKE_SOURCE_DIR}/src/nodes/lidar_sensor.cpp
        ${CMAKE_SOURCE_DIR}/src/nodes/lidar.cpp
        ${CMAKE_SOURCE_DIR}/src/algorithms/imu.cpp
        ${CMAKE_SOURCE_DIR}/src/nodes/imu.cpp
        ${CMAKE_SOURCE_DIR}/src/algorithms/aruco.cpp
        ${CMAKE_SOURCE_DIR}/src/nodes/aruco.cpp
        ${CMAKE_SOURCE_DIR}/src/nodes/maze_node.cpp
        ${CMAKE_SOURCE_DIR}/src/helper.cpp
)

set(AMENT_LIBS rclcpp std_msgs sensor_msgs cv_bridge image_transport)

add_executable(${PROJECT_NAME} src/main.cpp ${SRC})
target_include_directories(${PROJECT_NAME} PUBLIC
        include
        ${OpenCV_INCLUDE_DIRS}
        ${SDL2_INCLUDE_DIRS}

        # CHOOSE RIGHT ROS2 DISTRO NAME (school PC: humble)
        #/opt/ros/humble/include
        /opt/ros/jazzy/include
)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${SDL2_LIBS})
target_link_libraries(${PROJECT_NAME} atomic)
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs sensor_msgs cv_bridge image_transport visualization_msgs)

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
ament_package()

add_subdirectory(tests)
