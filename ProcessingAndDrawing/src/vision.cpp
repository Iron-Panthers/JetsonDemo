#include "vision.hpp"
#include <math.h>
using namespace std;
using namespace cv;

#define PI 3.14159265

float p = 0.0;   // angle of camera relative to ground // 0.2788 for real robit
float W = 640.0; // width of each frame
float H = 480.0; // height of each frame
float s_x = 60.0/180.0 * PI; // FOV in x axis of camera in radians (measure this)
float focal_length = (W/2) / tan(s_x/2); // focal length in "pixels"
float s_y = 2 * atan(H / (2 * focal_length)); // FOV in y axis of camera  in radians

bool sortContour(contour_type a, contour_type b)
{
    double aArea = contourArea(a, FALSE);
    double bArea = contourArea(b, FALSE);
    return aArea > bArea;
}

vector<contour_type> findContours(const Mat &bgr, Scalar minHSV, Scalar maxHSV, float minDensity) {
    //convert to hsv
    Mat hsvMat;
    cvtColor(bgr, hsvMat, COLOR_BGR2HSV);

    //threshold
    Mat threshedImg;
    inRange(hsvMat, minHSV, maxHSV, threshedImg);

    //find contours
    vector<contour_type> contours;
    vector<Vec4i> hierarchy; //throwaway, needed for function
    findContours(threshedImg, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

    // sort by area in descending order
    std::sort(contours.begin(), contours.end(), sortContour);

    // remove any vectors with invalid density
    for (int i = static_cast<int>(contours.size()); i >= 0; i--) // decrement for no conflict
    {
        contour_type cont = contours[i];
        double totalArea = contourArea(cont, FALSE);
        RotatedRect rect = minAreaRect(cont);
        double rectArea = rect.size.width * rect.size.height;
        double density = totalArea / rectArea; // compare area of the contour to the area of its bounding rect
        if (density < MIN_DENSITY) {
            contours.erase(contours.begin()+i);
        }
    }

    return contours;
}

VisionResultsPackage calculate(const Mat &bgr){

    ui64 time_began = millis_since_epoch();
    VisionResultsPackage res;

    

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