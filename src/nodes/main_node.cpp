#include "main_node.hpp"
#include "helper.hpp"


namespace nodes {
    MainNode::MainNode(
        std::shared_ptr<IoNode> io_node,
        std::shared_ptr<LineNode> line,
        std::shared_ptr<KinematicsNode> kinematics,
        std::shared_ptr<UltrasoundNode> ultrasound,
        std::shared_ptr<KeyboardInputNode> keyboard_input,
        std::shared_ptr<LidarNode> lidar_node,
        std::shared_ptr<ImuNode> imu_node,
        std::shared_ptr<MazeNode> maze_node
        ): Node("MainNode") {
       line_= line;
       kinematics_ = kinematics;
        ultrasound_ = ultrasound;
        keyboard_input_ = keyboard_input;
        lidar_node_ = lidar_node;
        imu_node_ = imu_node;
        button_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
        Topic::ionode_buttons, 1, std::bind(&MainNode::button_callback, this, std::placeholders::_1));
        keyboard_subscriber_ = this->create_subscription<std_msgs::msg::Char>(
        Topic::keyboardIn, 1, std::bind(&MainNode::keyboard_callback, this, std::placeholders::_1));
        maze_node_ = maze_node;
        io_node_ = io_node;
    }

    MainNode::~MainNode() {
        // Destructor Implementation (if needed)
    }

    void MainNode::FollowLine() {
        // TODO
    }

    void MainNode::keyboard_callback(std_msgs::msg::Char_<std::allocator<void>>::SharedPtr msg) {
        switch(msg->data) {
            case 'w':
                this->kinematics_->motorSpeed(10, 10, true, [](bool success){});
                break;
            case 's':
                this->kinematics_->motorSpeed(-10, -10, true, [](bool success){});
                break;
            case 'a':
                this->kinematics_->motorSpeed(-5, 5, true, [](bool success){});
                break;
            case 'd':
                this->kinematics_->motorSpeed(5, -5, true, [](bool success){});
                break;
            case 'l':
                if (this->lidar_node_->get_sensors_mode() == LidarMode::None) {
                    this->lidarCallback = [this]() {
                        this->lidar_node_->start(true, [this](IntersectionType detectedIntersection) {
                            std::cout << "intersection" << std::endl;
                            // kinematics_->backward(200, 10, [this](bool success) {
                                //this->lidar_node_->center([this]() {
                                    //kinematics_->forward(300, 10, [this](bool success) {
                                        this->lidarCallback();
                                    //});
                                //});
                            // });
                        });
                    };
                    this->lidarCallback();
                }else {
                    this->lidar_node_->stop();
                }

                break;
            case 'c':
                if (this->lidar_node_->get_sensors_mode() == LidarMode::None) {
                    this->lidar_node_->center([this]() {
                });
                }else {
                    this->lidar_node_->stop();
                }

                break;
            case 'u':
                if (this->ultrasound_->get_sensors_mode() == UltrasoundMode::None) {
                    this->ultrasound_->calibrationStart();
                    this->ultrasound_->calibrationEnd(true);
                }else {
                    this->ultrasound_->stop();
                }

                break;
            case 'i':
                if (this->imu_node_->getMode() == ImuNodeMode::None) {
                    this->imu_node_->start();
                }else {
                    this->imu_node_->stop();
                }

                break;
            case 'b':
                this->kinematics_->backward(50, 10, [this](bool success) {});

                break;
            case 'f':
                this->kinematics_->forward(100, 14, [this](bool success) {});

                break;
            case 'm':
                this->maze_node_->start();

                break;
            case 'k':
                this->line_->forwardUntilLine([this]() {});
                break;
            case ' ':
                this->maze_node_->stop();
                this->kinematics_->stop([](bool success) {});
                this->lidar_node_->stop();
                this->ultrasound_->stop();
                this->imu_node_->stop();
                this->io_node_->set_all_leds_color(0,0,0);
                break;
        }
    }

    void MainNode::button_callback(std_msgs::msg::UInt8_<std::allocator<void>>::SharedPtr msg) {
        switch(msg->data) {
            case 0:
                if (this->ultrasound_->get_sensors_mode() == UltrasoundMode::None) {
                    this->ultrasound_->calibrationStart();
                    this->ultrasound_->calibrationEnd(true);
                }else {
                    this->ultrasound_->stop();
                }
                break;
            case 1:
                this->maze_node_->start();
                /*
                if (this->line_->get_sensors_mode() == SensorsMode::None) {
                    line_->calibrationStart();
                    kinematics_->angle(0.5, 5,  [this](bool result) {
                        this->kinematics_->angle(-1, 5, [this](bool result) {
                            this->kinematics_->angle(0.5, 5,  [this](bool result) {
                                this->line_->calibrationEnd(false);
                                std::cout << "Finished moving forward" << std::endl;
                            });
                        });
                    });
                } else {
                    this->line_->stop();
                }
                */
                break;
            case 2:
                this->maze_node_->stop();
                this->kinematics_->stop([](bool success) {});
                this->lidar_node_->stop();
                this->ultrasound_->stop();
                this->imu_node_->stop();
                this->io_node_->set_all_leds_color(0,0,0);
                /*
                if (this->line_->get_sensors_mode() == SensorsMode::None) {
                    line_->calibrationStart();
                    kinematics_->angle(0.5, 5,  [this](bool result) {
                        this->kinematics_->angle(-1, 5, [this](bool result) {
                            this->kinematics_->angle(0.5, 5,  [this](bool result) {
                                this->line_->calibrationEnd(true);
                                std::cout << "Finished moving forward" << std::endl;
                            });
                        });
                    });
                } else {
                    this->line_->stop();
                }
                */
                break;
            default: break;
        }
    }

    
}
