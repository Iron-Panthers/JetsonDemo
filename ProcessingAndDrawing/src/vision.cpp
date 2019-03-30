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
float focal_length = (W/2) / tan(s_x/2);

float iData[9] = {559.57544781, 0.0, 287.88704838, 0.0, 558.25711671, 253.80973506, 0.0, 0.0, 1.0};
float dData[5] = {-0.0614758, -0.10311039, 0.00629035, -0.00674438, 0.47869622};
Mat intrinsic = Mat(Size(3,3), CV_32F, iData);
Mat distCoeffs = Mat(Size(1,5), CV_32F, dData);

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

    outerLeft.y -= H/2;
    outerRight.y -= H/2;
    topLeft.y -= H/2;
    topRight.y -= H/2;

    outerLeft.x -= W / 2;
    outerRight.x -= W / 2;
    topLeft.x -= W / 2;
    topRight.x -= W / 2;

    float a1 = atan(outerLeft.y/focal_length) - atan(topLeft.y/focal_length);
    float a2 = atan(outerRight.y / focal_length) - atan(topRight.y / focal_length);

    float centerX = (cx1 + cx2) / 2.0;
    float thetaPixels = centerX - W/2;
    float theta = thetaPixels * s_x / W;

    cout << "a1 = " << a1 << ", a2 = " << a2 << endl;

    float r1 = h / tan(a2 - p);
    float r2 = h / tan(a1 - p);

    cout << "r1 = " << r1 << ", r2 = " << r2 << endl;

    float t = atan(topRight.x / focal_length) - atan(topLeft.x / focal_length);
    cout << "t = " << t << endl;
    float bx = r1*cos(t);
	float by = r1*sin(t);
	float cx = r2;
	float cy = 0;

    float sAB = sqrt(bx*bx + by*by);
	float sAC = sqrt(cx*cx + cy*cy);
	float sBC = sqrt((cx-bx)*(cx-bx) + (cy-by)*(cy-by));

    float aB = acos((bx*(bx-cx)+by*(by-cy)) / (sAB*sBC));
	float aC = acos((cx*(cx-bx)+cy*(cy-by)) / (sAC*sBC));

	float y1 = r1*sin(aB);
	float y2 = r2*sin(aC);

	float x1 = w/2 - r1*cos(aB);
	float x2 = r2*cos(aC) - w/2;

    Point2f resultPoint;
	resultPoint.y = (y1+y2)/2.0;
	resultPoint.x = (x1+x2)/2.0;

    cout << resultPoint.x << ", " << resultPoint.y << ", " << theta << endl;

    //create the results package
    res.valid = true;
    res.timestamp = time_began;
    res.robotAngle = theta;
    res.robotPos = resultPoint;
    return res;
}