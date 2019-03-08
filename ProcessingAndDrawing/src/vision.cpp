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

float h = 5.2; //5.14192; //0.50076; // height in inches of tape that we compare to
float w = 11.739; //  width in inches of target that we compare to

float p = 0.0;   // angle of camera relative to ground // 0.2788 for real robit
float W = 640.0; // width of each frame
float H = 480.0; // height of each frame
float s_x = 60.0/180.0 * PI; // FOV in x axis of camera in radians
float s_y = s_x /  W * H; // FOV in y axis of camera  in radians

float iData[9] = {559.57544781, 0.0, 287.88704838, 0.0, 558.25711671, 253.80973506, 0.0, 0.0, 1.0};
float dData[5] = {-0.0614758, -0.10311039, 0.00629035, -0.00674438, 0.47869622};
Mat intrinsic = Mat(Size(3,3), CV_32F, iData);
Mat distCoeffs = Mat(Size(1,5), CV_32F, dData);

float a = 14.5 / 180.0 * PI;
float l = 5.0;

Point3f topLeft3(-w / 2, 0, 0);
Point3f outerLeft3(-w/2 - l*sin(a), -l*cos(a), 0);
Point3f topRight3(w/2, 0, 0);
Point3f outerRight3(w/2 + l*sin(a), -l*cos(a), 0);
std::vector<Point3f> objPoints = {topLeft3, outerLeft3, topRight3, outerRight3};

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

    contour_type leftPoints;
    contour_type rightPoints;

    if (cx1 < cx2) {
        leftPoints = cont1;
        rightPoints = cont2;
        // approxPolyDP(cont1, leftPoints, 2, true);
        // approxPolyDP(cont2, rightPoints, 2, true);
    } else {
        leftPoints = cont2;
        rightPoints = cont1;
        // approxPolyDP(cont2, leftPoints, 2, true);
        // approxPolyDP(cont1, rightPoints, 2, true);
    }

    int leftSize = static_cast<int>(leftPoints.size());
    int rightSize = static_cast<int>(rightPoints.size());

    float leftX[leftSize];
    float leftY[leftSize];
    float rightX[rightSize];
    float rightY[rightSize];
    for (int i = 0; i < leftSize; i++) {
        leftX[i] = leftPoints[i].x;
        leftY[i] = leftPoints[i].y;
    }
    for (int i = 0; i < rightSize; i++)
    {
        rightX[i] = rightPoints[i].x;
        rightY[i] = rightPoints[i].y;
    }

    int leftMinY = leftY[distance(leftY, min_element(leftY, leftY+leftSize))] + pos_thresh;
    int rightMinY = rightY[distance(rightY, min_element(rightY, rightY+rightSize))] + pos_thresh;
    int leftMinX = leftX[distance(leftX, min_element(leftX, leftX+leftSize))] + pos_thresh;
    int rightMaxX = rightX[distance(rightX, max_element(rightX, rightX+rightSize))] - pos_thresh;

    Point2f topLeft(W, 0);
    Point2f outerLeft(0, 0);
    for (int i = 0; i < leftSize; i++) {
        Point2f p = leftPoints[i];
        if (p.y <= leftMinY && p.x < topLeft.x) {
            topLeft = p;
        }
        if (p.x <= leftMinX && p.y > outerLeft.y) {
            outerLeft = p;
        }
    }

    Point2f topRight(0, 0);
    Point2f outerRight(0, 0);
    for (int i = 0; i < rightSize; i++)
    {
        Point2f p = rightPoints[i];
        if (p.y <= rightMinY && p.x > topRight.x)
        {
            topRight = p;
        }
        if (p.x >= rightMaxX && p.y > outerRight.y)
        {
            outerRight = p;
        }
    }

    Mat processedImage = bgr.clone();
    processedImage.at<Vec3b>(topLeft) = Vec3b(0, 255, 0);
    processedImage.at<Vec3b>(outerLeft) = Vec3b(0, 255, 0);
    processedImage.at<Vec3b>(topRight) = Vec3b(0, 255, 0);
    processedImage.at<Vec3b>(outerRight) = Vec3b(0, 255, 0);
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

    std::vector<Point2f> imgPoints = {topLeft, outerLeft, topRight, outerRight};
    solvePnP(objPoints, imgPoints, intrinsic, distCoeffs, last.rvec, last.tvec, true, CV_P3P);
    Mat R(3,3,CV_64F);
    Rodrigues(last.rvec, R);
    R = R.t();
    Mat pos = -R * last.tvec;
    Point3d posPoint(pos);

    // cout << last.rvec << endl;
    // cout << last.tvec << endl;

    //create the results package
    res.valid = true;
    res.timestamp = time_began;
    res.robotAngle = theta;
    res.robotPos.x = posPoint.x;
    res.robotPos.y = posPoint.z;

    cout << posPoint << endl;
    // cout << res.robotPos.x << ", " << res.robotPos.y << ", " << theta << endl;

    return res;
}