// mazeNode.cpp
// BPC-PRP project 2025
// xvarec06 & xruzic56
//
// Source file for maze escape mission node.


#include "mazeNode.hpp"
#include "helper.hpp"

uint32_t halfBlock = 200;

namespace nodes {
    MazeNode::MazeNode(
            std::shared_ptr<IoNode> ionode,
            std::shared_ptr<KinematicsNode> kinematics,
            std::shared_ptr<LidarNode> lidar_node,
            std::shared_ptr<ImuNode> imu_node,
            std::shared_ptr<LineNode> line_node
    ): rclcpp::Node ("maze_node"){

        this->lidar_node_ = lidar_node;
        this->imu_node_ = imu_node;
        this->kinematics_ = kinematics;
        this->ionode_ = ionode;
        this->line_node_ = line_node;
        this->aruco_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
                  Topic::aruco, 1, std::bind(&MazeNode::aruco_callback_, this, std::placeholders::_1));
    }

    MazeNode::~MazeNode() {}

    void MazeNode::stop(){
        kinematics_->stop();
        lidar_node_->stop();
        imu_node_->stop();
        this->arucoMutex.lock();
        arucoTreasure = ArucoTurn::None;
        arucoExit = ArucoTurn::None;
        this->arucoMutex.unlock();
        ionode_->set_all_leds_color(128,0,0);
    }

    void MazeNode::start(){
        this->arucoMutex.lock();
        arucoTreasure = ArucoTurn::None;
        arucoExit = ArucoTurn::None;
        this->arucoMutex.unlock();

        this->lidar_node_->start();
        this->imu_node_->start();

        this->fsmCallback = [this]() {

            // default strategy: turn left
            ArucoTurn wantedTurn = ArucoTurn::Left;

            // if aruco exists, use it (treasure prioritised)
            this->arucoMutex.lock();
                if(arucoTreasure != ArucoTurn::None) wantedTurn = arucoTreasure;
                else if(arucoExit != ArucoTurn::None) wantedTurn = arucoExit;
                arucoExit = ArucoTurn::None; // consume
                arucoTreasure = ArucoTurn::None; // consume
            this->arucoMutex.unlock();

            // probe surroundings with lidar, make decision, continue driving
            IntersectionType detectedIntersection = this->lidar_node_->getIntersectionInfo();
            switch(detectedIntersection) {
                case IntersectionType::I:
                    this->ionode_->led_blink(3, 200); // blink forward (LED on index 3)
                    this->kinematics_->forward(200, 10, [this](bool success) {this->fsmCallback();});
                    //imu_node_->forward(); // go 1 tile forward
                    // repeat -- call itself -- always --> after switch statement
                    break;

                case IntersectionType::U:
                    this->ionode_->led_blink(0, 200); // blink backward (LED on index 0)
                    this->imu_node_->turnBack(); // turn
                    this->kinematics_->forward(200, 10, [this](bool success) {this->fsmCallback();});
                    //imu_node_->forward(); // go 1 tile forward
                    break;

                case IntersectionType::RightTurn:
                    this->ionode_->led_blink(1, 200); // blink right (LED on index 1)
                    this->imu_node_->turnRight(); // turn
                    this->kinematics_->forward(200, 10, [this](bool success) {this->fsmCallback();});
                    //imu_node_->forward(); // go 1 tile forward
                    break;

                case IntersectionType::LeftTurn:
                    this->ionode_->led_blink(2, 200); // blink left (LED on index 2)
                    this->imu_node_->turnLeft(); // turn
                    this->kinematics_->forward(200, 10, [this](bool success) {this->fsmCallback();});
                    //imu_node_->forward(); // go 1 tile forward
                    break;

                case IntersectionType::RightT:                   
                    executeWantedTurn(wantedTurn); // blink and turn
                    this->kinematics_->forward(200, 10, [this](bool success) {this->fsmCallback();});
                    //imu_node_->forward(); // go 1 tile forward
                    break;

                case IntersectionType::LeftT:                    
                    executeWantedTurn(wantedTurn); // blink and turn
                    this->kinematics_->forward(200, 10, [this](bool success) {this->fsmCallback();});
                    //imu_node_->forward(); // go 1 tile forward
                    break;

                case IntersectionType::TopT:
                    executeWantedTurn(wantedTurn); // blink and turn
                    this->kinematics_->forward(200, 10, [this](bool success) {this->fsmCallback();});
                    //imu_node_->forward(); // go 1 tile forward
                    break;

                case IntersectionType::AllFour:
                    executeWantedTurn(wantedTurn); // blink and turn
                    this->kinematics_->forward(200, 10, [this](bool success) {this->fsmCallback();});
                    //imu_node_->forward(); // go 1 tile forward
                    break;

                case IntersectionType::F:
                    this->ionode_->led_blink(3, 200); // blink forward (LED on index 3)
                    this->kinematics_->forward(200, 10, [this](bool success) {this->fsmCallback();});
                    //imu_node_->forward(); // go 1 tile forward
                    break;

                case IntersectionType::None:
                default:
                    this->ionode_->led_blink(3, 200); // blink forward (LED on index 3)
                    this->kinematics_->forward(50, 10, [this](bool success) {this->fsmCallback();});
                    break;
            }
            
            //this->fsmCallback(); // repeat

            /*
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
                    kinematics_->forward(halfBlock, 14, [this](bool success)
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
                    });
                }
            }); */
        };
        this->fsmCallback();
    }

    // apply current strategy on intersection
    void MazeNode::executeWantedTurn(ArucoTurn wantedTurn) {
        switch(wantedTurn) {
            case ArucoTurn::Left:
                this->ionode_->led_blink(2, 200); // blink left (LED on index 2)
                this->imu_node_->turnLeft();
                this->imu_node_->forward();
                break;

            case ArucoTurn::Right:
                this->ionode_->led_blink(1, 200); // blink right (LED on index 1)
                this->imu_node_->turnRight();
                this->imu_node_->forward();
                break;

            case ArucoTurn::Forward:
                this->ionode_->led_blink(3, 200); // blink forward (LED on index 3)
                this->imu_node_->forward();
                break;

            case ArucoTurn::None:
            default:
                // should not happen; 2x longer blink forward
                this->ionode_->led_blink(3, 300);
                this->ionode_->led_blink(3, 300);
                this->imu_node_->stop();
                break;
        }
    }

    // Save next direction from decoded aruco tag
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
