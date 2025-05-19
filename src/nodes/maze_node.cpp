//
// Created by root on 4/23/25.
//
#include "mazeNode.hpp"
#include "helper.hpp"

uint32_t halfBlock = 90;

namespace nodes {
    MazeNode::MazeNode(
            std::shared_ptr<IoNode> ionode,
            std::shared_ptr<KinematicsNode> kinematics,
            std::shared_ptr<LidarNode> lidar_node,
            std::shared_ptr<ImuNode> imu_node,
            std::shared_ptr<UltrasoundNode> ultrasound_node,
            std::shared_ptr<LineNode> line_node
    ): rclcpp::Node ("maze_node"){

        this->lidar_node_ = lidar_node;
        this->imu_node_ = imu_node;
        this->kinematics_ = kinematics;
        this->ionode_ = ionode;
        this->ultrasound_node_ = ultrasound_node;
        this->line_node_ = line_node;
        this->aruco_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
                  Topic::aruco, 1, std::bind(&MazeNode::aruco_callback_, this, std::placeholders::_1));
    }

    MazeNode::~MazeNode() {}

    void MazeNode::stop(){
        this->arucoMutex.lock();
        arucoTreasure = ArucoTurn::None;
        arucoExit = ArucoTurn::None;
        this->arucoMutex.unlock();
    }

    void MazeNode::start(){
        arucoTreasure = ArucoTurn::None;
        arucoExit = ArucoTurn::None;
        this->lidarCallback = [this]() {
            this->lidar_node_->start(true, [this](IntersectionType detectedIntersection) {
                kinematics_->stop();
                if (detectedIntersection == IntersectionType::U)
                {
                    this->kinematics_->turnBack(10, [this](bool sucess) {
                        IntersectionType detectedIntersection = lidar_node_->getThisIntersection();
                        if (detectedIntersection == IntersectionType::None)
                        {
                            this->ionode_->showIntersection(IntersectionType::None);
                        }
                        this->lidarCallback();
                    });
                }else{
                    this->ultrasound_node_->extremeTestingStart([this](bool sucess)
                    {
                        this->kinematics_->interruptOp();
                        this->ultrasound_node_->handleExtreme([this]()
                        {
                            this->kinematics_->continueOp();
                        }, UltrasoundDirection::Front);
                    });
                    kinematics_->forward(80, 14, [this](bool success)
                    {
                        this->ultrasound_node_->stop();
                        IntersectionType detectedIntersection = lidar_node_->getThisIntersection();
                        this->arucoMutex.lock();
                        ArucoTurn wantedTurn = arucoExit;
                        arucoExit = ArucoTurn::None;
                        arucoTreasure = ArucoTurn::None;
                        this->arucoMutex.unlock();

                        if (wantedTurn == ArucoTurn::Right and (detectedIntersection == IntersectionType::RightT
                        or detectedIntersection == IntersectionType::AllFour
                        or detectedIntersection == IntersectionType::TopT)){
                            this->kinematics_->turnRight(10, [this](bool sucess) {
                                IntersectionType detectedIntersection = lidar_node_->getThisIntersection();
                                this->lidarCallback();
                            });
                        } else if (wantedTurn == ArucoTurn::Left and (detectedIntersection == IntersectionType::LeftT
                            or detectedIntersection == IntersectionType::AllFour
                            or detectedIntersection == IntersectionType::TopT)){
                            this->kinematics_->turnLeft(10, [this](bool sucess) {
                                IntersectionType detectedIntersection = lidar_node_->getThisIntersection();
                                this->lidarCallback();
                            });
                        } else if (wantedTurn == ArucoTurn::Forward and (detectedIntersection == IntersectionType::RightT
                            or detectedIntersection == IntersectionType::AllFour
                            or detectedIntersection == IntersectionType::LeftT)){
                            IntersectionType detectedIntersection = lidar_node_->getThisIntersection();
                            this->lidarCallback();
                        } else if (detectedIntersection == IntersectionType::RightT) {
                            this->kinematics_->turnRight(10, [this](bool sucess) {
                                IntersectionType detectedIntersection = lidar_node_->getThisIntersection();
                                this->lidarCallback();
                            });
                        } else if (detectedIntersection == IntersectionType::LeftT) {
                            IntersectionType detectedIntersection = lidar_node_->getThisIntersection();
                            this->lidarCallback();
                        } else if (detectedIntersection == IntersectionType::TopT) {
                            this->kinematics_->turnRight(10, [this](bool sucess) {
                                IntersectionType detectedIntersection = lidar_node_->getThisIntersection();
                                this->lidarCallback();
                            });
                        } else if (detectedIntersection == IntersectionType::AllFour)
                        {
                            this->kinematics_->turnRight(10, [this](bool sucess) {
                                IntersectionType detectedIntersection = lidar_node_->getThisIntersection();
                                this->lidarCallback();
                            });
                        } else if (detectedIntersection == IntersectionType::LeftTurn){
                            this->kinematics_->turnLeft(10, [this](bool sucess) {
                                // IntersectionType detectedIntersection = lidar_node_->getThisIntersection();
                                this->lidarCallback();
                            });
                        } else if (detectedIntersection == IntersectionType::RightTurn){
                            this->kinematics_->turnRight(10, [this](bool sucess) {
                                // IntersectionType detectedIntersection = lidar_node_->getThisIntersection();
                                this->lidarCallback();
                            });
                        }else {
                            IntersectionType detectedIntersection = lidar_node_->getThisIntersection();
                            this->lidarCallback();
                        }
                        // kinematics_->forward(300, 10, [this](bool success) {
                        // });
                    });
                }
            });
        };
        this->lidarCallback();
    }

    void MazeNode::aruco_callback_(std_msgs::msg::UInt8_<std::allocator<void>>::SharedPtr msg){
        this->arucoMutex.lock();
        uint8_t data = msg->data;
        if (data == 0){
            this->arucoExit = ArucoTurn::Forward;
        }else if (data == 1){
            this->arucoExit = ArucoTurn::Left;
        }else if (data == 2){
            this->arucoExit = ArucoTurn::Right;
        } else if (data == 10){
            this->arucoTreasure = ArucoTurn::Forward;
        }else if (data == 11){
            this->arucoTreasure = ArucoTurn::Left;
        }else if (data == 12){
            this->arucoTreasure = ArucoTurn::Right;
        }
        this->arucoMutex.unlock();
    }
}
