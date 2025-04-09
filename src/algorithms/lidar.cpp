//
// Created by root on 4/9/25.
//

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
        std::vector<float> left{};
        std::vector<float> right{};
        std::vector<float> front{};
        std::vector<float> back{};

        // TODO: Define how wide each directional sector should be (in radians)
        constexpr float angle_range = M_PI / 2 ;

        // Compute the angular step between each range reading
        auto angle_step = (angle_end - angle_start) / points.size();

        for (size_t i = 0; i < points.size(); ++i) {
            if(points[i] != std::numeric_limits<float>::infinity() and points[i] != -std::numeric_limits<float>::infinity()) {
                auto angle = angle_start + i * angle_step+M_PI;
                if ((angle < ((angle_range) + 1.0/4.0 * M_PI)) and (angle > 1.0/4.0 * M_PI)) { // 1/4 PI - 3/4 PI
                    left.push_back(points[i]);
                }else if (angle < ((2*angle_range) + 1.0/4.0 * M_PI)) { // 3/4 PI - 5/4 PI
                    back.push_back(points[i]);
                }else if (angle < ((3*angle_range) + 1.0/4.0 * M_PI)) { // 5/4 PI - 7/4 PI
                    right.push_back(points[i]);
                }else{  // 7/4 PI - 1/4 PI
                    front.push_back(points[i]);
                }
            }
            // TODO: Skip invalid (infinite) readings
            // TODO: Sort the value into the correct directional bin based on angle
        }



        // TODO: Return the average of each sector (basic mean filter)
        return LidarFiltrResults{
            .front = vectorMean(front),
            .back = vectorMean(back),
            .left = vectorMean(left),
            .right = vectorMean(right),
        };
    }
} // algorithms