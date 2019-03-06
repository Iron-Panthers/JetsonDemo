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
float minArea = 300;
float maxAreaRatio = 8;
float minDensity = 0.75;

float h = 5.14192; // height in inches of tape that we compare to
float w = 11.739; //  width in inches of target that we compare to

float p = 0.0;   // angle of camera relative to ground // 0.2788 for real robit
float W = 640.0; // width of each frame
float H = 480.0; // height of each frame
float s_x = 63.0/180.0 * PI; // FOV in x axis of camera in radians
float s_y = s_x /  W * H; // FOV in y axis of camera  in radians

float iData[9] = {559.57544781, 0.0, 287.88704838, 0.0, 558.25711671, 253.80973506, 0.0, 0.0, 1.0};
float dData[5] = {-0.0614758, -0.10311039, 0.00629035, -0.00674438, 0.47869622};

Mat intrinsic = Mat(Size(3,3), CV_32F, iData);
Mat distCoeffs = Mat(Size(1,5), CV_32F, dData);

float a = 14.5 / 180.0 * PI;
float l = 2.0;

Point3f topLeft3(-w/2, 0, 0);
Point3f innerLeft3(-w/2 + l * cos(a), -l *sin(a), 0);
Point3f topRight3(w/2, 0, 0);
Point3f innerRight3(w/2 - l * cos(a), -l *sin(a), 0);
std::vector<Point3f> objPoints = {topLeft3,innerLeft3,topRight3,innerRight3};

VisionResultsPackage last;

bool sortContour(contour_type a, contour_type b)
{
    double aArea = contourArea(a, FALSE);
    double bArea = contourArea(b, FALSE);
    return aArea > bArea;
}

VisionResultsPackage calculate(const Mat &bgr){

    ui64 time_began = millis_since_epoch();
    VisionResultsPackage res;

    // blur(bgr, bgr, Size(3, 3));

    //convert to hsv
    Mat hsvMat;
    cvtColor(bgr, hsvMat, COLOR_BGR2HSV);

    //threshold on green (light ring color), any saturation, high brightness
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

    // find 2 biggest contours with area > minArea
    int contoursSize = static_cast<int>(contours.size());

    RotatedRect cont1;
    float cont1Area = 0;
    bool found1 = false;
    RotatedRect cont2;
    bool found2 = false;
    for (int i = 0; i < contoursSize; i++) {
        contour_type cont = contours[i];
        float totalArea = contourArea(cont, FALSE);
        RotatedRect rect = minAreaRect(cont);
        float hullArea = rect.size.width * rect.size.height;
        float density = totalArea / hullArea;
        if (totalArea > minArea && density > minDensity) {
            if (!found1) {
                found1 = true;
                cont1 = rect;
                cont1Area = totalArea;
            } else if (!found2 && cont1Area / totalArea < maxAreaRatio) {
                found2 = true;
                cont2 = rect;
                i = contoursSize;
            }
        } else if (totalArea < minArea) {
            i = contoursSize;
        }
    }

    if (!found1 || !found2) { //  if we ended without both contours we exit
        return res; // with failure result
    }

    float theta = ((cont1.center.x + cont2.center.x) / 2.0) * s_x / W;

    Point2f leftRect[4];
    Point2f rightRect[4];
    if (cont1.center.x < cont2.center.x)
    {
        cont1.points(leftRect);
        cont2.points(rightRect);
    }
    else
    {
        cont2.points(leftRect);
        cont1.points(rightRect);
    }
    float leftX[4];
    float leftY[4];
    float rightX[4];
    float rightY[4];
    for (int i = 0; i < 4; i++)
    {
        leftX[i] = leftRect[i].x;
        leftY[i] = leftRect[i].y;
        rightX[i] = rightRect[i].x;
        rightY[i] = rightRect[i].y;
    }

    int leftMinY = distance(leftY, min_element(leftY, leftY + 4));
    int rightMinY = distance(rightY, min_element(rightY, rightY + 4));
    int leftMaxX = distance(leftX, max_element(leftX, leftX + 4));
    int rightMinX = distance(rightX, min_element(rightX, rightX + 4));

    Point2f topLeft = leftRect[leftMinY];
    Point2f innerLeft = leftRect[leftMaxX];
    Point2f topRight = rightRect[rightMinY];
    Point2f innerRight = rightRect[rightMinX];

    Mat processedImage = threshedImg.clone();
    threshold(processedImage, processedImage, 0, 255, THRESH_BINARY);
    circle(processedImage, topLeft, 2, Scalar(190, 190, 190), 2);
    circle(processedImage, innerLeft, 2, Scalar(190, 190, 190), 2);
    circle(processedImage, topRight, 2, Scalar(190, 190, 190), 2);
    circle(processedImage, innerRight, 2, Scalar(190, 190, 190), 2);
    cvtColor(processedImage, processedImage, CV_GRAY2BGR);
    if (imgCount == 30)
    {
        vector<int> compression_params;
        compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
        compression_params.push_back(9);
        imwrite("/tmp/image.png", processedImage, compression_params);
        imgCount = 0;
    }
    else
    {
        imgCount++;
    }

    std::vector<Point2f> imgPoints = {topLeft, innerLeft, topRight, innerRight};
    solvePnP(objPoints, imgPoints, intrinsic, distCoeffs, last.rvec, last.tvec, true, CV_P3P);
    Mat R(3,3,CV_64F);
    Rodrigues(last.rvec, R);
    R = R.t();
    Mat pos = -R * last.tvec;
    Point3d posPoint(pos);

    // cout << last.rvec << endl;
    cout << last.tvec << endl;

    //create the results package
    res.valid = true;
    res.timestamp = time_began;
    res.robotAngle = theta;
    res.robotPos.x = posPoint.x;
    res.robotPos.y = posPoint.z;

    // cout << posPoint << endl;
    // cout << res.robotPos.x << ", " << res.robotPos.y << ", " << theta << endl;

    return res;
}