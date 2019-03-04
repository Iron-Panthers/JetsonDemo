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

bool sortContour(contour_type a, contour_type b)
{
    double aArea = contourArea(a, FALSE);
    double bArea = contourArea(b, FALSE);
    return aArea > bArea;
}

VisionResultsPackage calculate(const Mat &bgr){

    ui64 time_began = millis_since_epoch();
    VisionResultsPackage res;

    Mat bgrFixed = Mat(bgr.size(), CV_8UC3);
    undistort(bgr, bgrFixed, intrinsic, distCoeffs);

    //convert to hsv
    Mat hsvMat;
    cvtColor(bgrFixed, hsvMat, COLOR_BGR2HSV);

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
    RotatedRect rect1;
    bool foundRect1 = false;
    RotatedRect rect2;
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
                rect1 = rect;
            } else {
                foundRect2 = true;
                rect2 = rect;
                i = numContours; // if we found both contours we're done
            }
        }
    }

    if (!foundRect1 || !foundRect2) { //  if we ended without both contours we exit
        return res; // with failure result
    }

    Point2f leftRect[4];
    Point2f rightRect[4];
    if (rect1.center.x < rect2.center.x) {
        rect1.points(leftRect);
        rect2.points(rightRect);
    } else {
        rect2.points(leftRect);
        rect1.points(rightRect);
    }
    float leftX[4];
    float leftY[4];
    float rightX[4];
    float rightY[4];
    for (int i = 0; i < 4; i++) {
        leftX[i] = leftRect[i].x;
        leftY[i] = leftRect[i].y;
        rightX[i] = rightRect[i].x;
        rightY[i] = rightRect[i].y;
    }

    Mat processedImage = threshedImg.clone();
    threshold(processedImage, processedImage, 0, 255, THRESH_BINARY);
    for (int i = 0; i < 4; i++) {
        circle(processedImage, leftRect[i], 4, Scalar(255, 255, 255), 4);
        circle(processedImage, rightRect[i], 4, Scalar(255, 255, 255), 4);
    }
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

    int leftMinY = distance(leftY, min_element(leftY, leftY+4));
    int rightMinY = distance(rightY, min_element(rightY, rightY+4));
    int leftMinX = distance(leftX, min_element(leftX, leftX+4));
    int rightMaxX = distance(rightX, max_element(rightX, rightX+4));

    Point2f topLeft = leftRect[leftMinY];
    Point2f leftLeft = leftRect[leftMinX];
    Point2f topRight = rightRect[rightMinY];
    Point2f rightRight = rightRect[rightMaxX];

    float leftHeight = leftLeft.y - topLeft.y;
    float rightHeight = rightRight.y - topRight.y;
    float width = topRight.x - topLeft.x;
    float centerX = (rect1.center.x + rect2.center.x) / 2.0;
    float thetaPixels = centerX - W/2;
    float theta = thetaPixels * s_x / W;

    float a1 = leftHeight * s_y / H;
    float a2 = rightHeight * s_y / H;
    float r1 = h / tan(a2 - p);
    float r2 = h / tan(a1 - p);

    cout << theta << "," << width << endl;

    float t = width * s_x / W;
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

    // cout << resultPoint.x << ", " << resultPoint.y << ", " << theta << endl;

    //create the results package
    res.valid = true;
    res.timestamp = time_began;
    res.robotAngle = theta;
    res.robotPos = resultPoint;
    return res;
}