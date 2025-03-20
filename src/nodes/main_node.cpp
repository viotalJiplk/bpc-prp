#include "main_node.hpp"
#include "helper.hpp"


namespace nodes {
    MainNode::MainNode(
        std::shared_ptr<Motors> motors, 
        std::shared_ptr<LineNode> line, 
        std::shared_ptr<Encoder> encoders, 
        std::shared_ptr<IoNode> ionode
    ): Node("MainNode") {
       motors_= motors;
       line_= line;
       encoders_= encoders;
       ionode_= ionode;
       MainNode::button_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
        Topic::ionode_buttons, 1, std::bind(&MainNode::button_callback, this, std::placeholders::_1));
    }

    MainNode::~MainNode() {
        // Destructor Implementation (if needed)
    }

    void MainNode::FollowLine() {
        // TODO
    }

    void button_callback(std_msgs::msg::UInt8_<std::allocator<void>>::SharedPtr msg) {
        switch(msg->data) {
            case 0: MainNode::FollowLine(); break;
            case 1: break;
            case 2: break;
            default: break;
        }
    }

    
}
