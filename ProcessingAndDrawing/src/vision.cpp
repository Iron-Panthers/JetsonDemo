#include "vision.hpp"
#include <math.h>
using namespace std;
using namespace cv;

#define PI 3.14159265

int imgCount = 0;

Scalar cargoMinHSV = Scalar(10, 125, 200);
Scalar cargoMaxHSV = Scalar(20, 255, 255);
float cargoMinDensity = 0.6;  // a cirlce is 78% area of its bounding square
float cargoDiameter = 13.0;

Scalar hatchMinHSV = Scalar(105, 125, 40);
Scalar hatchMaxHSV = Scalar(120, 255, 255);
float hatchMinDensity = 0.6;  // a cirlce is 78% area of its bounding square
float hatchDiameter = 19.5; // 19.0 ish on real field
float hatchHeight = 26.75 + hatchDiameter/2;  //19.0 - hatchDiameter/2.0; ish on real field

    float W = 640.0;  // width of each frame
float H = 480.0;  // height of each frame
float s_x = 60.0 / 180.0 * PI; // FOV in x axis of camera in radians (measure this)
float focal_length = (W / 2) / tan(s_x / 2);  // focal length in "pixels"
float s_y = 2 * atan(H / (2 * focal_length));  // FOV in y axis of camera  in radians
float camera_angle = -0.335; // angle of camera above horizontal
float camera_height = 46.0;

bool sortContour(contour_type a, contour_type b) {
    float aArea = contourArea(a, FALSE);
    float bArea = contourArea(b, FALSE);
    return aArea > bArea;
}

/*
xPixelAngle: Returns the horizontal angle from the eye line of the camera to the pixel
	float pixelX: the raw x value of the pixel
	returns: the angle in radians, positive to the right of the eye line
*/
float xPixelAngle(float pixelX) {
	return atan((pixelX - W/2) / focal_length);
}

/*
yPixelAngle: Returns the vertical angle from the eye line of the camera to the pixel
	float pixelY: the raw y value of the pixel
	returns: the angle in radians, positive above the eye line
*/
float yPixelAngle(float pixelY) {
	return atan((pixelY - H/2) / focal_length);
}

/*
findContours: Finds the contours on the given image with the given limits
	const mat &bgr: input Mat
	Scalar minHSV: minimum hue, saturation, and value
	Scalar maxHSV: maximum hue, saturation, and value
	float minDensity: minimum ratio of contour area to bounding rect area
	bool write: true to write image every 30 frames, false if not
	returns: a vector of contours, sorted by area, descending
*/
vector<contour_type> findContours(const Mat &bgr, Scalar minHSV, Scalar maxHSV, float minDensity, bool write) {
    // convert to hsv
    Mat hsvMat;
    cvtColor(bgr, hsvMat, COLOR_BGR2HSV);

    // threshold
    Mat threshedImg;
    inRange(hsvMat, minHSV, maxHSV, threshedImg);

    // find contours
    vector<contour_type> contours;
    vector<Vec4i> hierarchy;  // throwaway, needed for function
    findContours(threshedImg, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

    // sort by area in descending order
    std::sort(contours.begin(), contours.end(), sortContour);

    // remove any vectors with invalid density
    for (int i = static_cast<int>(contours.size()) - 1; i >= 0; i--)  // decrement for no conflict
    {
        contour_type cont = contours[i];
        float totalArea = contourArea(cont, FALSE);
        RotatedRect rect = minAreaRect(cont);
        float rectArea = rect.size.width * rect.size.height;
        float density = totalArea / rectArea;  // compare area of the contour to
                                                // the area of its bounding rect
        if (density < minDensity) {
            contours.erase(contours.begin() + i);
        }
    }

	// every 30 frames, write the most recent frame to an image
    if (write && static_cast<int>(contours.size()) > 0 && imgCount % 30 == 0) {
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

/*
distanceToWall: Finds distance to a target that is in a fixed position on a wall
	float pixelY: the raw y value of the pixel which we will be finding the distance to
	float groundToPixel: the distance from the ground to the location, in real life, in inches
	returns: distance to the target, in inches
*/
float distanceToWall(float pixelY, float groundToPixel) {
	float angle = yPixelAngle(pixelY);
    float d = (groundToPixel - camera_height) / tan(angle + camera_angle); // found this formula, and then it also came from limelight docs
	return d;
}

/*
angleToWall: Finds the horizontal angle to the wall, indepedent of camera look direction
	float leftPixelX: the raw x value of the pixel on the left of our vision target
	float rightPixelX: the raw x value of the pixel on the right of our vision target
	float r: the distance, in inches, to the vision target
	float h: the width, in inches, of the vision target in real life
	returns: the angle relative to the wall, in radians, always positive
*/
float angleToWall(float leftPixelX, float rightPixelX, float r, float h) {
	float leftA = xPixelAngle(leftPixelX);
	float rightA = xPixelAngle(rightPixelX);
	float a = leftA - rightA; // angle between pixels

	if (cos(a) == 0) { // if cos(a) == 0 formula will break, so just return 0 :(
		return 0;
	}

	// magic formula! Uses law of cosines, assumes that r is the distance to the center of the hatch
	float term1 = (4*r*r + h*h)/(4*r*h);
	float term2 = (4*r*r - h*h)/(4*r*h*cos(a));
	float det = term1*term1 - term2*term2;
	if (abs(det) > 1) { // limit det to <1 so that acos doesn't break
		det = copysign(1, det);
	}
	float angle = acos(sqrt(det));

	return angle;
}

VisionResultsPackage calculate(const Mat &bgr, cv::Mat &processedImage) {
    imgCount++;
	cout << "Image " << imgCount;
    ui64 time_began = millis_since_epoch();
    VisionResultsPackage res;

	processedImage = bgr.clone();

    vector<contour_type> cargo = findContours(bgr, cargoMinHSV, cargoMaxHSV, cargoMinDensity, false);
	vector<contour_type> hatch = findContours(bgr, hatchMinHSV, hatchMaxHSV, hatchMinDensity, true);

    if (static_cast<int>(cargo.size()) == 0) {
        cout << " | No cargo found.";
    } else {
        drawContours(processedImage, cargo, 0, Scalar(0, 255, 0), 2);

        Rect cargoRect = boundingRect(cargo[0]);
        float rectX = cargoRect.x + cargoRect.width / 2 - W / 2;
        float rectY = cargoRect.y + cargoRect.height / 2 - H / 2;

        // find angle to cargo
        float angleToCargo = xPixelAngle(rectX + W / 2);

        // find distance to cargo
        float screenRadius = (cargoRect.width + cargoRect.height) / 4.0;
        float screenDistance = sqrt(rectX * rectX + rectY * rectY);
        float smallAngle = atan((screenDistance - screenRadius) / focal_length);
        float bigAngle = atan((screenDistance + screenRadius) / focal_length);
        float screenAngle = bigAngle - smallAngle;
        float distanceToCargo = cargoDiameter / (2.0 * tan(screenAngle / 2.0));

    	cout << " | Angle: " << angleToCargo << ", Distance: " << distanceToCargo;

        res.cargoAngle = angleToCargo;
        res.cargoDistance = distanceToCargo;
    }

    if (static_cast<int>(hatch.size()) == 0) {
		cout << " | No hatch found.";
	} else {
		drawContours(processedImage, hatch, 0, Scalar(0, 255, 0), 2);

		Rect hatchRect = boundingRect(hatch[0]);
		vector<Point> bestHatch = hatch[0];

		// x location of midpoint of hatch used to get angle
		float midX = hatchRect.x + hatchRect.width/2;
		float angleToHatch = xPixelAngle(midX);

		// y location of midpoint of hatch used to get distance
		float yMidpoint = hatchRect.y + hatchRect.height/2;
		float r = distanceToWall(yMidpoint, hatchHeight);

		// use left and right side of hatch to get angle relative to wall
		float leftX = hatchRect.x;
		float rightX = hatchRect.x + hatchRect.width;
		float angle = angleToWall(leftX, rightX, r, hatchDiameter);

		res.hatchAngle = angleToHatch;
		res.hatchDistance = r;
		res.hatchWallAngle = angle;
		res.hatchX = r * cos(angle);
		res.hatchY = r * sin(angle);

    	cout << " | Angle: " << res.hatchAngle << ", Distance: " << res.hatchDistance << ", Wall Angle: " << res.hatchWallAngle << ", X: " << res.hatchX << ", Y: " << res.hatchY;
	}

	cout << endl;

    // create the results package
    res.cargoValid = true;
    res.timestamp = time_began;
    return res;
}
