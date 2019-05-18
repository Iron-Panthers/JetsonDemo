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

Scalar hatchMinHSV = Scalar(105, 125, 50);
Scalar hatchMaxHSV = Scalar(120, 255, 255);
float hatchMinDensity = 0.6;  // a cirlce is 78% area of its bounding square
float hatchDiameter = 19.0;
float hatchHeight = 19.0 - hatchDiameter/2.0;

float W = 640.0;  // width of each frame
float H = 480.0;  // height of each frame
float s_x =
    60.0 / 180.0 * PI;                        // FOV in x axis of camera in radians (measure this)
float focal_length = (W / 2) / tan(s_x / 2);  // focal length in "pixels"
float s_y =
    2 * atan(H / (2 * focal_length));  // FOV in y axis of camera  in radians
float camera_angle = 0.0; // angle of camera above horizontal
float camera_height = 35.0;

bool sortContour(contour_type a, contour_type b) {
    float aArea = contourArea(a, FALSE);
    float bArea = contourArea(b, FALSE);
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
        float totalArea = contourArea(cont, FALSE);
        RotatedRect rect = minAreaRect(cont);
        float rectArea = rect.size.width * rect.size.height;
        float density = totalArea / rectArea;  // compare area of the contour to
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

float distanceToWall(float pixelY, float groundToPixel) {
	float angle = atan((H/2.0 - pixelY) / focal_length);
    float d = (groundToPixel - camera_height) / tan(angle + camera_angle);
	return d;
}

float angleToWall(float leftPixelX, float rightPixelX, float r, float h) {
	float leftA = atan((leftPixelX - W/2) / focal_length);
	float rightA = atan((rightPixelX - W/2) / focal_length);
	float a = leftA - rightA;

	float term1 = (4*r*r + h*h)/(4*r*h);
	float term2 = (4*r*r - h*h)/(4*r*h*cos(a));
	float det = term1*term1 - term2*term2;
	if (det < 0) {
		det = 0;
	}
	if (abs(det) > 1) {
		det = copysign(1, det);
	}
	float angle = acos(sqrt(det));

	return angle;
}

VisionResultsPackage calculate(const Mat &bgr) {
    imgCount++;
	cout << "Image " << imgCount;
    ui64 time_began = millis_since_epoch();
    VisionResultsPackage res;

    vector<contour_type> cargo = findContours(bgr, cargoMinHSV, cargoMaxHSV, cargoMinDensity);
	vector<contour_type> hatch = findContours(bgr, hatchMinHSV, hatchMaxHSV, hatchMinDensity);

    if (static_cast<int>(cargo.size()) == 0) {
        cout << " | No cargo found.";
    } else {
		Rect cargoRect = boundingRect(cargo[0]);
		float rectX = cargoRect.x + cargoRect.width/2 - W / 2;
		float rectY = cargoRect.y + cargoRect.height/2 - H / 2;

		// find angle to cargo
		float angleToCargo = atan(rectX / focal_length);

		// find distance to cargo
		float screenRadius = (cargoRect.width + cargoRect.height) / 4.0;
		float screenDistance = sqrt(rectX*rectX + rectY*rectY);
		float smallAngle = atan((screenDistance - screenRadius) / focal_length);
		float bigAngle = atan((screenDistance + screenRadius) / focal_length);
		float screenAngle = bigAngle - smallAngle;
		float distanceToCargo = cargoDiameter / 2.0 / tan(screenAngle / 2.0);

    	cout << " | Angle: " << angleToCargo << ", Distance: " << distanceToCargo;

        res.cargoAngle = angleToCargo;
        res.cargoDistance = distanceToCargo;
    }

    if (static_cast<int>(hatch.size()) == 0) {
		cout << " | No hatch found.";
	} else {
		Rect hatchRect = boundingRect(hatch[0]);

		float hatchX = hatchRect.x + hatchRect.width/2 - W/2;
		float angleToHatch = atan(hatchX / focal_length);

		float yMidpoint = hatchRect.y + hatchRect.height/2;
		float r = distanceToWall(yMidpoint, hatchHeight);

		float leftX = hatchRect.x;
		float rightX = hatchRect.x + hatchRect.width;
		float angle = angleToWall(leftX, rightX, r, hatchDiameter);

		res.hatchAngle = angleToHatch;
		res.hatchDistance = r;
		res.hatchWallAngle = angle;
		res.hatchX = r * cos(angle);
		res.hatchY = r * sin(angle);
	}

    // create the results package
    res.cargoValid = true;
    res.timestamp = time_began;
    return res;
}
