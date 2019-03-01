#ifndef VISION_HPP
#define VISION_HPP

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <iostream>
#include <utility>
#include "helper.hpp"

using namespace std;

struct VisionResultsPackage {
    i64 timestamp;
    bool valid = false;
	cv::Point robotPos;
	double robotAngle;

    static string createCSVHeader () {
        return "Timestamp, Valid, X, Y, Theta";
    }

    string createCSVLine () {
        if (!valid) {
            stringstream ss;
            ss << timestamp << "," << valid << ",0,0,0";
            return ss.str();
        } else {
            stringstream ss;
            ss << timestamp << "," << valid << "," << robotPos.x << "," << robotPos.y << "," << robotAngle;
            return ss.str();
        }
    }
};

typedef std::vector<cv::Point> contour_type;

/**
 * Processes the raw image provided in order to determine interesting values
 * from the image. Results package is returned in a struct.
 * @param bgr raw image to do processing on
 * @return results of vision processing (e.g location of target, timestamp)
 */ 
VisionResultsPackage calculate(const cv::Mat &bgr);

#endif