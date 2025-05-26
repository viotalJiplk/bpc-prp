// lidar.cpp
// BPC-PRP project 2025
// xvarec06 & xruzic56
//
// Source file for LiDAR data filtering algorithm.


#include "lidar.hpp"
#include <cmath>

#include <numeric>

namespace algorithms {

    LidarFiltr::LidarFiltr(){};
    LidarFiltr::~LidarFiltr(){};

    float LidarFiltr::vectorMean(std::vector<float> vector){
        float sum = 0.0f;
        for (float num : vector) {
            sum += num;
        }
        return sum/vector.size();
    }

    LidarFiltrResults LidarFiltr::apply_filter(std::vector<float> points, float angle_start, float angle_end) {

        // Create containers for values in different directions
        std::vector<float> front {};
        std::vector<float> front_left {};
        std::vector<float> front_right {};
        std::vector<float> back {};
        std::vector<float> back_left {};
        std::vector<float> back_right {};
        std::vector<float> left {};
        std::vector<float> right {};

        // Define how wide each directional sector should be (in radians)
        constexpr float angle_range = M_PI / 4 ;

        // Compute the angular step between each range reading
        auto angle_step = (angle_end - angle_start) / points.size();
        for (size_t i = 0; i < points.size(); ++i) {
            float point = points.at(i);
            // Skip invalid (infinite) readings
            if (point == -std::numeric_limits<float>::infinity() or point == std::numeric_limits<float>::infinity()) {
                point = 0;
            }
            // Sort the value into the correct directional bin based on angle
            auto angle = angle_start + (i * angle_step) + M_PI;
            if ((angle < ((angle_range) + 3.0/8.0 * M_PI)) and (angle > 3.0/8.0 * M_PI)) { // 0/8 PI - 2/8 PI
                left.push_back(point);
            }else if (angle < ((2*angle_range) + 3.0/8.0 * M_PI) and (angle > 3.0/8.0 * M_PI)) { // 2/8 PI - 4/8 PI
                back_left.push_back(point); //
            }else if (angle < ((3*angle_range) + 3.0/8.0 * M_PI) and (angle > 3.0/8.0 * M_PI)) { // 4/8 PI - 6/8 PI
                back.push_back(point); //
            }else if (angle < ((4*angle_range) + 3.0/8.0 * M_PI) and (angle > 3.0/8.0 * M_PI)) { // 6/8 PI - 8/8 PI
                back_right.push_back(point);
            }else if (angle < ((5*angle_range) + 3.0/8.0 * M_PI) and (angle > 3.0/8.0 * M_PI)) { // 8/8 PI - 10/8 PI
                right.push_back(point);
            }else if (angle < ((6*angle_range) + 3.0/8.0 * M_PI) and (angle > 3.0/8.0 * M_PI)) { // 10/8 PI - 12/8 PI
                front_right.push_back(point);
            }else if (angle < ((7*angle_range) + 3.0/8.0 * M_PI) and (angle > 3.0/8.0 * M_PI)) { // 12/8 PI - 14/8 PI
                front.push_back(point);
            }else{                                                  // 14/8 PI - 16/8 PI
                front_left.push_back(point); //
            }
        }



        // Return the average of each sector (basic mean filter)
        return LidarFiltrResults{
            .front = vectorMean(front),
            .front_left = vectorMean(front_left),
            .front_right = vectorMean(front_right),
            .back = vectorMean(back),
            .back_left = vectorMean(back_left),
            .back_right = vectorMean(back_right),
            .left = vectorMean(left),
            .right = vectorMean(right),
        };
    }
} // algorithms
