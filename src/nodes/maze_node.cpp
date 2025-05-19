//
// Created by root on 4/23/25.
//
#include "mazeNode.hpp"
#include "helper.hpp"

namespace nodes {
    MazeNode::MazeNode(
            std::shared_ptr<IoNode> ionode,
            std::shared_ptr<KinematicsNode> kinematics,
            std::shared_ptr<LidarNode> lidar_node,
            std::shared_ptr<ImuNode> imu_node
    ): rclcpp::Node ("maze_node"){

      this->lidar_node_ = lidar_node;
      this->imu_node_ = imu_node;
      this->kinematics_ = kinematics;
      this->ionode_ = ionode;
        this->aruco_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
                  Topic::aruco, 1, std::bind(&MazeNode::aruco_callback_, this, std::placeholders::_1));
    }

    MazeNode::~MazeNode() {}

    void MazeNode::start(){
        this->arucoMutex.lock();
        while(!this->arucoExitQueue.empty()){
            this->arucoExitQueue.pop();
        }
        while(!this->arucoTreasureQueue.empty()){
            this->arucoTreasureQueue.pop();
        }
        this->arucoMutex.unlock();
        this->lidarCallback = [this]() {
            this->lidar_node_->start(true, [this](IntersectionType detectedIntersection) {
                kinematics_->stop();
                kinematics_->forward(150, 14, [this](bool success) {
                    IntersectionType detectedIntersection = lidar_node_->getThisIntersection();
                    this->arucoMutex.lock();
                    ArucoTurn wantedTurn = ArucoTurn::None;
                    if (!this->arucoExitQueue.empty()){
                        wantedTurn = this->arucoExitQueue.front();
                        this->arucoExitQueue.pop();
                    }
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
                    } else if (detectedIntersection == IntersectionType::AllFour) {
                        this->kinematics_->turnRight(10, [this](bool sucess) {
                            IntersectionType detectedIntersection = lidar_node_->getThisIntersection();
                            this->lidarCallback();
                        });
                    } else {
                        IntersectionType detectedIntersection = lidar_node_->getThisIntersection();
                        this->lidarCallback();
                    }
                    // kinematics_->forward(300, 10, [this](bool success) {
                    // });
                });
            });
        };
        this->lidarCallback();
    }
    void MazeNode::aruco_callback_(std_msgs::msg::UInt8_<std::allocator<void>>::SharedPtr msg){
        this->arucoMutex.lock();
        uint8_t data = msg->data;
        if (data == 0){
            this->arucoExitQueue.push(ArucoTurn::Forward);
        }else if (data == 1){
            this->arucoExitQueue.push(ArucoTurn::Left);
        }else if (data == 2){
            this->arucoExitQueue.push(ArucoTurn::Right);
        } else if (data == 10){
            this->arucoTreasureQueue.push(ArucoTurn::Forward);
        }else if (data == 11){
            this->arucoTreasureQueue.push(ArucoTurn::Left);
        }else if (data == 12){
            this->arucoTreasureQueue.push(ArucoTurn::Right);
        }
        this->arucoMutex.unlock();
    }
}
