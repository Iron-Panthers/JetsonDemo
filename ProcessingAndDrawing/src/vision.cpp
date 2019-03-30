#include "vision.hpp"
#include <math.h>
using namespace std;
using namespace cv;

#define PI 3.14159265

int imgCount = 0;

int MIN_HUE = 0; //55;
int MAX_HUE = 180; //90;

int MIN_SAT = 0;
int MAX_SAT = 255;

int MIN_VAL = 220;
int MAX_VAL = 255;

Scalar threshMin = Scalar(MIN_HUE, MIN_SAT, MIN_VAL);
Scalar threshMax = Scalar(MAX_HUE, MAX_SAT, MAX_VAL);

float MIN_DENSITY = 0.75;
int CONTOURS_TO_CHECK = 5;
int pos_thresh = 0;

float W = 640.0; // width of each frame
float s_x = 60.0/180.0 * PI; // FOV in x axis of camera in radians

bool sortContour(contour_type a, contour_type b)
{
    double aArea = contourArea(a, FALSE);
    double bArea = contourArea(b, FALSE);
    return aArea > bArea;
}

VisionResultsPackage calculate(const Mat &bgr){

    ui64 time_began = millis_since_epoch();
    VisionResultsPackage res;

    // Mat bgrFixed = Mat(bgr.size(), CV_8UC3);
    // undistort(bgr, bgrFixed, intrinsic, distCoeffs);

    //convert to hsv
    Mat hsvMat;
    cvtColor(bgr, hsvMat, COLOR_BGR2HSV);

    //threshold on green (light ring color), high saturation, high brightness
    Mat threshedImg;
    inRange(hsvMat, threshMin, threshMax, threshedImg);

    //find contours
    vector<contour_type> contours;
    vector<Vec4i> hierarchy; //throwaway, needed for function
    try {
        findContours(threshedImg, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
    }
    catch (...) { 
        return res; //return the default result (failure)
    }

    // sort by size in descending order
    std::sort(contours.begin(), contours.end(), sortContour);

    // go thru the top CONTOURS_TO_CHECK contours, or as many as possible if we don't have that many
    int contoursSize = static_cast<int>(contours.size());
    // cout << "n contours  " << contoursSize << endl;
    int numContours = std::min(CONTOURS_TO_CHECK, contoursSize);
    contour_type cont1;
    bool foundRect1 = false;
    contour_type cont2;
    bool foundRect2 = false;
    for (int i = 0; i < numContours; i++) {
        contour_type cont = contours[i];
        double totalArea = contourArea(cont, FALSE);
        // cout << "contour " << i << " area = " << totalArea << endl;
        RotatedRect rect = minAreaRect(cont);
        double rectArea = rect.size.width * rect.size.height;
        double density = totalArea / rectArea; // compare area of the contour to the area of its bounding rect
        if (density >= MIN_DENSITY) {
            if (!foundRect1) {
                foundRect1 = true;
                cont1 = cont;
            } else {
                foundRect2 = true;
                cont2 = cont;
                i = numContours; // if we found both contours we're done
            }
        }
    }

    if (!foundRect1 || !foundRect2) { //  if we ended without both contours we exit
        return res; // with failure result
    }

    Moments m1 = moments(cont1);
    Moments m2 = moments(cont2);
    float cx1 = m1.m10 / m1.m00;
    float cx2 = m2.m10 / m2.m00;
    float thetaPixels = (cx1 + cx2)/2.0 - W / 2;
    float theta = thetaPixels * s_x / W;

    //create the results package
    res.valid = true;
    res.timestamp = time_began;
    res.robotAngle = theta;
    res.robotPos.x = 0;
    res.robotPos.y = 0;

    return res;
}