#include "vision.hpp"
using namespace std;
using namespace cv;

int imgCount = 0;

inline int getHue (Mat &img, int r, int c) {
    return img.at<Vec3b>(r, c)[0];
}

inline int getSat (Mat &img, int r, int c) {
    return img.at<Vec3b>(r, c)[1];
}

inline int getVal (Mat &img, int r, int c) {
    return img.at<Vec3b>(r, c)[2];
}

VisionResultsPackage calculate(const Mat &bgr){
    ui64 time_began = millis_since_epoch();

    if (imgCount == 0) {
        vector<int> compression_params;
        compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
        compression_params.push_back(9);
        imwrite("/tmp/image.png", bgr, compression_params);
        imgCount++;
    }

    //create the results package
    VisionResultsPackage res;
    res.valid = true;
    res.timestamp = time_began;
    return res;
}
