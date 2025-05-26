#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include "aruco.hpp"


namespace algorithms {

    ArucoDetector::ArucoDetector() {
        // Initialize dictionary with 4x4 markers (50 possible IDs)
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    }

    ArucoDetector::~ArucoDetector(){};

    // Detect markers in the input image
    std::vector<ArucoDetector::Aruco> ArucoDetector::detect(cv::Mat frame) {
        std::vector<ArucoDetector::Aruco> arucos;

        std::vector<uint8_t> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;

        // Detect markers using OpenCV
        cv::aruco::detectMarkers(frame, ArucoDetector::dictionary_, marker_corners, marker_ids);

        if (!marker_ids.empty()) {
            // std::cout << "Arucos found: ";
            for (size_t i = 0; i < marker_ids.size(); i++) {
                // std::cout << (int)marker_ids.at(i) << " ";

                // Create Aruco struct and add to result vector
                ArucoDetector::Aruco aruco;
                aruco.id = marker_ids.at(i);
                aruco.corners = marker_corners.at(i);
                arucos.emplace_back(aruco);
            }
            // std::cout << std::endl;
        }
        else {
            // std::cout << "Arucos not found" << std::endl;
        }

        return arucos;
    }




}
