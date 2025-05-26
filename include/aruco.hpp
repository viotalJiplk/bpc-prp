// aruco.hpp
// BPC-PRP project 2025
// xvarec06 & xruzic56
//
// Header file for ArUco marker detection algorithm.


#ifndef ARUCO_HPP
#define ARUCO_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

namespace algorithms {

    class ArucoDetector {
    public:

     // Represents one detected marker
     struct Aruco {
         uint8_t id;
         std::vector<cv::Point2f> corners;
     };

     ArucoDetector();
     ~ArucoDetector();

     // Detect markers in the input image
     std::vector<Aruco> detect(cv::Mat frame);

    private:
        cv::Ptr<cv::aruco::Dictionary> dictionary_;
    };
}

#endif //ARUCO_HPP
