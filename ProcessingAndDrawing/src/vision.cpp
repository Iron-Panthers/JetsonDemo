#include "vision.hpp"
#include <math.h>
using namespace std;
using namespace cv;

#define PI 3.14159265

int imgCount = 0;

Scalar cargoMinHSV = Scalar(0, 100, 120);
Scalar cargoMaxHSV = Scalar(30, 255, 255);
float cargoMinDensity = 0.6;  // a cirlce is 78% area of its bounding square

float p = 0.0;    // angle of camera relative to ground // 0.2788 for real robit
float W = 640.0;  // width of each frame
float H = 480.0;  // height of each frame
float s_x =
    60.0 / 180.0 * PI;                        // FOV in x axis of camera in radians (measure this)
float focal_length = (W / 2) / tan(s_x / 2);  // focal length in "pixels"
float s_y =
    2 * atan(H / (2 * focal_length));  // FOV in y axis of camera  in radians

bool sortContour(contour_type a, contour_type b) {
    double aArea = contourArea(a, FALSE);
    double bArea = contourArea(b, FALSE);
    return aArea > bArea;
}

vector<contour_type> findContours(const Mat &bgr, Scalar minHSV, Scalar maxHSV,
                                  float minDensity) {
    // convert to hsv
    Mat hsvMat;
    cvtColor(bgr, hsvMat, COLOR_BGR2HSV);

    // threshold
    Mat threshedImg;
    inRange(hsvMat, minHSV, maxHSV, threshedImg);

    // find contours
    vector<contour_type> contours;
    vector<Vec4i> hierarchy;  // throwaway, needed for function
    findContours(threshedImg, contours, hierarchy, RETR_TREE,
                 CHAIN_APPROX_SIMPLE);

    // sort by area in descending order
    std::sort(contours.begin(), contours.end(), sortContour);

    // remove any vectors with invalid density
    for (int i = static_cast<int>(contours.size()) - 1; i >= 0; i--)  // decrement for no conflict
    {
        contour_type cont = contours[i];
        double totalArea = contourArea(cont, FALSE);
        RotatedRect rect = minAreaRect(cont);
        double rectArea = rect.size.width * rect.size.height;
        double density = totalArea / rectArea;  // compare area of the contour to
                                                // the area of its bounding rect
        if (density < minDensity) {
            contours.erase(contours.begin() + i);
        }
    }

    if (static_cast<int>(contours.size()) > 0 && imgCount % 30 == 0) {
        Mat processedImage = threshedImg.clone();
        threshold(processedImage, processedImage, 0, 255, THRESH_BINARY);
        cvtColor(processedImage, processedImage, CV_GRAY2BGR);
        vector<int> compression_params;
        compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
        compression_params.push_back(9);
        imwrite("/tmp/threshImg.png", processedImage, compression_params);
        imwrite("/tmp/realImg.png", bgr, compression_params);
    }

    return contours;
}

VisionResultsPackage calculate(const Mat &bgr) {
    imgCount++;
    ui64 time_began = millis_since_epoch();
    VisionResultsPackage res;

    vector<contour_type> cargo =
        findContours(bgr, cargoMinHSV, cargoMaxHSV, cargoMinDensity);

    if (static_cast<int>(cargo.size()) == 0) {
        cout << "No target, Image " << imgCount << endl;
        return res;
    }

    Rect cargoRect = boundingRect(cargo[0]);
    float rectX = cargoRect.x + cargoRect.width - W / 2;

    // find angle to cargo
    double angleToCargo = atan(rectX / focal_length);

    cout << "Image " << imgCount << ", Angle: " << angleToCargo << endl;

    // create the results package
    res.cargoValid = true;
    res.timestamp = time_began;
    res.cargoAngle = angleToCargo;
    res.cargoDistance = 10;
    return res;
}
