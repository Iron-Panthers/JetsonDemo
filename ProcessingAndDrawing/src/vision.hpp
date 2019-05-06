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
    bool hatchValid = false;
    bool holeValid = false;
	double hatchAngle;
    double hatchDisplace;
    double holeAngle;
    double holeDisplace;

    static string createCSVHeader () {
        return "Timestamp, Valid, HatchValid, HatchAngle, HatchDisplace, HoleValid, HoleAngle, HoleDisplace";
    }

    string createCSVLine () {
        stringstream ss;
        ss << timestamp << ",";

        if (hatchValid) {
            ss << "1," << hatchAngle << "," << hatchDisplace << ",";
        } else {
            ss << "0,0,0,";
        }
        
        if (holeValid) {
            ss << "1," << holeAngle << "," << holeDisplace;
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
VisionResultsPackage calculate(const cv::Mat &bgr);

/**
 * Creates neural net from given names, config, weights file
 */
void createNet(string namesFile, string cfgFile, string weightsFile);

#endif