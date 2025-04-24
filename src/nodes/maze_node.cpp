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

    }

    MazeNode::~MazeNode() {}

    void MazeNode::start(){
        this->lidarCallback = [this]() {
            this->lidar_node_->start(true, [this](IntersectionType detectedIntersection) {
                std::cout << "intersection type: " << std::endl;
                kinematics_->stop();
                if (detectedIntersection == IntersectionType::RightT) {
                    this->kinematics_->angle(-M_PI/2.0, 10, [this](bool sucess) {
                        this->lidarCallback();
                    });
                } else if (detectedIntersection == IntersectionType::LeftT) {
                    this->lidarCallback();
                } else if (detectedIntersection == IntersectionType::TopT) {
                    this->kinematics_->angle(-M_PI/2.0, 10, [this](bool sucess) {
                        this->lidarCallback();
                    });
                } else if (detectedIntersection == IntersectionType::AllFour) {
                    this->kinematics_->angle(-M_PI/2.0, 10, [this](bool sucess) {
                        this->lidarCallback();
                    });
                } else {
                    this->lidarCallback();
                }
                // kinematics_->forward(300, 10, [this](bool success) {
                // });
            });
        };
        this->lidarCallback();
    }
}
