//
// Created by root on 4/9/25.
//

#ifndef LIDAR_H
#define LIDAR_H

#endif //LIDAR_H
#include <vector>

namespace algorithms {
    struct LidarFiltrResults {
        float front;
        float front_left;
        float front_right;
        float back;
        float back_left;
        float back_right;
        float left;
        float right;
    };
    class LidarFiltr {
        public:
            LidarFiltr();
            ~LidarFiltr();
            float vectorMean(std::vector<float> vector);
            LidarFiltrResults apply_filter(std::vector<float> points, float angle_start, float angle_end);
    };
}