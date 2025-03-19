//
// Created by root on 3/19/25.
//
#include "line_node.hpp"
#include "helper.hpp"

namespace nodes {
    LineNode::LineNode(): Node("LineNode") {
        line_sensors_subscriber_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
                  Topic::line_sensors, 1, std::bind(&LineNode::on_line_sensors_msg, this, std::placeholders::_1));
    }

    LineNode::~LineNode() {
        // Destructor Implementation (if needed)
    }

    float LineNode::get_continuous_line_pose() const{
      return 0.0;
    }
    DiscreteLinePose LineNode::get_discrete_line_pose() const{
        return DiscreteLinePose::LineNone;
    }

    void LineNode::on_line_sensors_msg(std::shared_ptr<std_msgs::msg::UInt16MultiArray> msg){
        left_sensor = msg->data[0];
        right_sensor = msg->data[1];
    }

    float LineNode::estimate_continuous_line_pose(float left_value, float right_value){
        return 0.0;
    }

    DiscreteLinePose LineNode::estimate_descrete_line_pose(float l_norm, float r_norm) {
        float result = l_norm - r_norm;
        if(result > 0.2){
            return DiscreteLinePose::LineOnLeft;
        }else if(result < -0.2){
            return DiscreteLinePose::LineOnRight;
        }else if(l_norm > 0.2 && r_norm > 0.2){
            return DiscreteLinePose::LineBoth;
        }else{
            return DiscreteLinePose::LineNone;
        }
    }
}