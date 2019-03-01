#include "vision.hpp"
#include <math.h>
using namespace std;
using namespace cv;

#define PI 3.14159265

int imgCount = 0;

int MIN_HUE = 70;
int MAX_HUE = 100;

int MIN_SAT = 100;
int MAX_SAT = 255;

int MIN_VAL = 150;
int MAX_VAL = 255;

Scalar threshMin = Scalar(MIN_HUE, MIN_SAT, MIN_VAL);
Scalar threshMax = Scalar(MAX_HUE, MAX_SAT, MAX_VAL);

int MIN_DENSITY = 0.75;
int CONTOURS_TO_CHECK = 5;

float h = 5.14192; // height in inches of tape that we compare to
float w = 11.739; //  width in inches of target that we compare to

float p = 0; // angle of camera relative to ground
float W = 640; // width of each frame
float H = 480; // height of each frame
float s_x = 75/180 * PI; // FOV in x axis of camera in radians
float s_y = s_x /  W * H; // FOV in y axis of camera  in radians

bool sortContour(contour_type a, contour_type b)
{
    double aArea = contourArea(a, FALSE);
    double bArea = contourArea(b, FALSE);
    return aArea > bArea;
}

VisionResultsPackage calculate(const Mat &bgr){
    ui64 time_began = millis_since_epoch();
    VisionResultsPackage res;

    //blur the image
    blur(bgr, bgr, Size(3, 3));

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

    Mat processedImage = threshedImg.clone();
    threshold(processedImage, processedImage, 0, 255, THRESH_BINARY);
    cvtColor(processedImage, processedImage, CV_GRAY2BGR);
    if (imgCount == 30) {
        vector<int> compression_params;
        compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
        compression_params.push_back(9);
        imwrite("/tmp/image.png", processedImage, compression_params);
        imgCount = 0;
    }
    else {
        imgCount++;
    }

    if (contours.size() < 2) {
        return res; //return the default result (failure)
    }

    // sort by size in descending order
    std::sort(contours.begin(), contours.end(), sortContour);

    // go thru the top CONTOURS_TO_CHECK contours, or as many as possible if we don't have that many
    int contoursSize = static_cast<int>(contours.size());
    int numContours = std::min(CONTOURS_TO_CHECK, contoursSize);
    RotatedRect rect1;
    bool foundRect1 = false;
    RotatedRect rect2;
    bool foundRect2 = false;
    for (int i = 0; i < numContours; i++) {
        contour_type cont = contours[i];
        double totalArea = contourArea(cont, FALSE);
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

    int leftMinY = distance(leftY, min_element(leftY, leftY+4));
    int rightMinY = distance(rightY, min_element(rightY, rightY+4));
    int leftMinX = distance(leftX, min_element(leftX, leftX+4));
    int rightMaxX = distance(rightX, min_element(rightX, rightX+4));

    Point2f topLeft = leftRect[leftMinY];
    Point2f leftLeft = leftRect[leftMinX];
    Point2f topRight = rightRect[rightMinY];
    Point2f rightRight = rightRect[rightMaxX];

    float leftHeight = leftLeft.y - topLeft.y;
    float rightHeight = rightRight.y - topRight.y;
    float width = topRight.x - topLeft.x;
    float centerX = (rect1.center.x + rect2.center.y) / 2;
    float thetaPixels = centerX - W/2;

    float theta = thetaPixels * s_x / W;

    float a1 = leftHeight * s_y / H;
    float a2 = rightHeight * s_y / H;
    float r1 = h / tan(a2 - p);
    float r2 = h / tan(a1 - p);

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

    Point resultPoint;
	resultPoint.y = (y1+y2)/2;
	resultPoint.x = (x1+x2)/2;

    //create the results package
    res.valid = true;
    res.timestamp = time_began;
    res.robotAngle = theta;
    res.robotPos = resultPoint;
    return res;
}

/*
//checks for contour validity
bool is_valid (contour_type &contour) {
    bool valid = true; //start out assuming its valid, disprove this later

    //find bounding rect & convex hull
    cv::Rect rect = cv::boundingRect(contour);
    contour_type hull;
    cv::convexHull(contour, hull);

    double totalArea = (RES_X * RES_Y);

    //calculate relevant ratios & values
    double area = cv::contourArea(contour) / totalArea;
    //double perim = cv::arcLength(hull, true);

    double convex_area = cv::contourArea(hull) / totalArea;

    double width = rect.width, height = rect.height;

    double area_rat = area / convex_area;
    double rect_rat = height / width;

  //check ratios & values for validity
    if (area < MIN_AREA || area > MAX_AREA) valid = false;
    if (area_rat < MIN_AREA_RAT || area_rat > MAX_AREA_RAT) valid = false;
    if (rect_rat < MIN_RECT_RAT || rect_rat > MAX_RECT_RAT) valid = false;
    if (width < MIN_WIDTH || width > MAX_WIDTH) valid = false;
    if (height < MIN_HEIGHT || height > MAX_HEIGHT) valid = false;

    return valid;
}

VisionResultsPackage calculate(const cv::Mat &bgr, cv::Mat &processedImage){
    ui64 time_began = millis_since_epoch();
    //blur the image
    cv::blur(bgr, bgr, cv::Size(5,5));
    cv::Mat hsvMat;
    //convert to hsv
    cv::cvtColor(bgr, hsvMat, cv::COLOR_BGR2HSV);

    //store HSV values at a given test point to send back
    int hue = getHue(hsvMat, TEST_POINT.x, TEST_POINT.y);
    int sat = getSat(hsvMat, TEST_POINT.x, TEST_POINT.y);
    int val = getVal(hsvMat, TEST_POINT.x, TEST_POINT.y);

    //threshold on green (light ring color)
    cv::Mat greenThreshed;
    cv::inRange(hsvMat,
                cv::Scalar(MIN_HUE, MIN_SAT, MIN_VAL),
                cv::Scalar(MAX_HUE, MAX_SAT, MAX_VAL),
                greenThreshed);

    processedImage = greenThreshed.clone();
    cv::threshold (processedImage, processedImage, 0, 255, cv::THRESH_BINARY);
    cv::cvtColor(processedImage, processedImage, CV_GRAY2BGR); 
    //processedImage = bgr.clone();  

    drawPoint (processedImage, TEST_POINT, GUIDE_DOT);

    //contour detection
    vector<contour_type> contours;
    vector<cv::Vec4i> hierarchy; //throwaway, needed for function
    try {
        cv::findContours (greenThreshed, contours, hierarchy, 
            cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    }
    catch (...) { //TODO: change this to the error that occurs when there are no contours
        return processingFailurePackage(time_began);
    }

    if (contours.size() < 1) { //definitely did not find 
        return processingFailurePackage(time_began);
    }
    
    //store the convex hulls of any valid contours
    vector<contour_type> valid_contour_hulls;
    for (int i = 0; i < (int)contours.size(); i++) {
        contour_type contour = contours[i];
        if (is_valid (contour)) {
            contour_type hull;
            cv::convexHull(contour, hull);
            valid_contour_hulls.push_back(hull);
        }
    }

    int numContours = valid_contour_hulls.size();
    printf ("Num contours: %d\n", numContours);
    
    if (numContours < 1) { //definitely did not find 
        return processingFailurePackage(time_began);
    }

    //find the largest contour in the image
    contour_type largest;
    double largestArea = 0;
    for (int i = 0; i < numContours; i++){
        double curArea = cv::contourArea(valid_contour_hulls[i], true);
        if (curArea > largestArea){
            largestArea = curArea;
            largest = valid_contour_hulls[i];
        }
    }

    //get the points of corners
    vector<cv::Point> all_points;
    all_points.insert (all_points.end(), largest.begin(), largest.end());

    //find which corner is which
    cv::Point ul (1000, 1000), ur (0, 1000), ll (1000, 0), lr (0, 0);
    for (int i = 0; i < (int)all_points.size(); i++) {
        int sum = all_points[i].x + all_points[i].y;
        int dif = all_points[i].x - all_points[i].y;

        if (sum < ul.x + ul.y) {
            ul = all_points[i];
        }

        if (sum > lr.x + lr.y) {
            lr = all_points[i];
        }

        if (dif < ll.x - ll.y) {
            ll = all_points[i];
        }

        if (dif > ur.x - ur.y) {
            ur = all_points[i];
        }
    } 

    //find the center of mass of the largest contour
    cv::Moments centerMass = cv::moments(largest, true);
    double centerX = (centerMass.m10) / (centerMass.m00);
    double centerY = (centerMass.m01) / (centerMass.m00);
    cv::Point center (centerX, centerY);

    vector<contour_type> largestArr;
    largestArr.push_back(largest);
    cv::drawContours(processedImage, largestArr , 0, MY_GREEN, 2);

    double top_width = ur.x - ul.x;
    double bottom_width = lr.x - ll.x;
    double left_height = ll.y - ul.y;
    double right_height = lr.y - ur.y;

    //create the results package
    VisionResultsPackage res;
    res.timestamp = time_began;
    res.valid = true;
    
    copyPointData (ul, res.ul);
    copyPointData (ur, res.ur);
    copyPointData (ll, res.ll);
    copyPointData (lr, res.lr);
    copyPointData (center, res.midPoint);

    res.upperWidth = top_width;
    res.lowerWidth = bottom_width;
    res.leftHeight = left_height;
    res.rightHeight = right_height;

    res.sampleHue = hue;
    res.sampleSat = sat;
    res.sampleVal = val;

    drawOnImage (processedImage, res);
    return res;
}
*/
