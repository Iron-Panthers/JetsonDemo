#ifndef VISION_HPP
#define VISION_HPP

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <iostream>
#include <utility>
#include "helper.hpp"

struct VisionResultsPackage {
    i64 timestamp;
    bool hatchValid = false;
    bool cargoValid = false;
	double hatchAngle = 0;
    double hatchDistance = 0;
	double hatchWallAngle = 0;
	double hatchX = 0;
	double hatchY = 0;
    double cargoAngle = 0;
    double cargoDistance = 0;

    static string createCSVHeader () {
        return "Timestamp, HatchValid, HatchAngle, HatchDistance, HatchWallAngle, hatchX, hatchY, CargoValid, CargoAngle, CargoDistance";
    }

    string createCSVLine () {
        stringstream ss;
        ss << timestamp << ",";

        if (hatchValid) {
            ss << "1," << hatchAngle << "," << hatchDistance << ",";
			ss << hatchWallAngle << "," << hatchX << "," << hatchY << ",";
        } else {
            ss << "0,0,0,0,0,0,";
        }
        
        if (cargoValid) {
            ss << "1," << cargoAngle << "," << cargoDistance;
        } else {
            ss << "0,0,0";
        }

        return ss.str();
    }
};

typedef std::vector<cv::Point> contour_type;

/**
 * Processes the raw image provided in order to determine interesting values
 * from the image. Results package is returned in a struct.
 * @param bgr raw image to do processing on
 * @return results of vision processing (e.g location of target, timestamp)
 */
VisionResultsPackage calculate(const cv::Mat &bgr, cv::Mat &processedImage);

#endif