#ifndef VISION_HPP
#define VISION_HPP

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <iostream>
#include <utility>
#include "helper.hpp"

struct VisionResultsPackage {
    long long timestamp;
    bool valid = false;
	cv::Point robotPos;
	double robotAngle;
    cv::Mat rvec = cv::Mat(3, 3, CV_64F);
    cv::Mat tvec = cv::Mat(1, 3, CV_64F);

    static std::string createCSVHeader () {
        return "Timestamp, Valid, X, Y, Theta";
    }

    std::string createCSVLine () {
        if (!valid) {
            std::stringstream ss;
            ss << timestamp << "," << valid << ",0,0,0";
            return ss.str();
        } else {
            std::stringstream ss;
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