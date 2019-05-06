#include "vision.hpp"
#include <math.h>
using namespace std;
using namespace cv;

float confThreshold = 0.5; // Confidence threshold
float nmsThreshold = 0.4;  // Non-maximum suppression threshold
Net net;
vector<String> names;
vector<String> classes;

int MIN_HUE = 0;   //55;
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

float robotHeight = 46.5; //inches

float topHoleHeight = 26.25; //inches
float holeTopToCamera = robotHeight - topHoleHeight;
float holeHeight = 13; // inches
float holeBottomToCamera = holeTopToCamera + holeHeight;

float topHatchHeight = 28; //inches
float hatchTopToCamera = robotHeight - topHatchHeight;
float hatchHeight = 16.5; // inches
float hatchBottomToCamera = hatchTopToCamera + hatchHeight;

float p = 0.0;                 // angle of camera relative to ground // 0.2788 for real robit
float W = 640.0;               // width of each frame
float H = 480.0;               // height of each frame
float s_x = 60.0 / 180.0 * PI; // FOV in x axis of camera in radians
float s_y = s_x / W * H;       // FOV in y axis of camera  in radians
float focal_length = (W / 2) / tan(s_x / 2);

void createNet(string namesFile, string cfgFile, string weightsFile) {
    // Load names of classes
    ifstream ifs(namesFile.c_str());
    string line;
    while (getline(ifs, line)) {
        classes.push_back(line);
    }

    // Load the network
    net = readNetFromDarknet(cfgFile, weightsFile);
    net.setPreferableBackend(DNN_BACKEND_OPENCV);
    net.setPreferableTarget(DNN_TARGET_CPU);
}

// Get the names of the output layers
vector<String> getOutputsNames(const Net &net) {
    if (names.empty()) {
        //Get the indices of the output layers, i.e. the layers with unconnected outputs
        vector<int> outLayers = net.getUnconnectedOutLayers();

        //get the names of all the layers in the network
        vector<String> layersNames = net.getLayerNames();

        // Get the names of the output layers in names
        names.resize(outLayers.size());
        for (size_t i = 0; i < outLayers.size(); i++) {
            names[i] = layersNames[outLayers[i] - 1];
        }
    }
    return names;
}

// Remove the bounding boxes with low confidence using non-maxima suppression
void postprocess(Mat &frame, const vector<Mat> &outs, vector<string> &boxClasses, vector<float> &finalConfidences, vector<Rect> &finalBoxes) {

    vector<int> classIds;
    vector<float> confidences;
    vector<Rect> boxes;

    for (size_t i = 0; i < outs.size(); ++i) {
        // Scan through all the bounding boxes output from the network and keep only the
        // ones with high confidence scores. Assign the box's class label as the class
        // with the highest score for the box.
        float *data = (float *)outs[i].data;
        for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
        {
            Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
            Point classIdPoint;
            double confidence;
            // Get the value and location of the maximum score
            minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
            if (confidence > confThreshold)
            {
                int centerX = (int)(data[0] * frame.cols);
                int centerY = (int)(data[1] * frame.rows);
                int width = (int)(data[2] * frame.cols);
                int height = (int)(data[3] * frame.rows);
                int left = centerX - width / 2;
                int top = centerY - height / 2;

                classIds.push_back(classIdPoint.x);
                confidences.push_back((float)confidence);
                boxes.push_back(Rect(left, top, width, height));
            }
        }
    }

    // Perform non maximum suppression to eliminate redundant overlapping boxes with
    // lower confidences
    vector<int> indices;
    NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);

    if (int i = 0; i < indices.size(); i++) {
        int idx = indices[i];
        boxClasses.push_back(classes[classIds[idx]]);
        finalConfidences.push_back(confidences[idx]);
        finalBoxes.push_back(boxes[idx]);
    }

}

Rect bestHatch(vector<int> &boxClasses, vector<float> &confidences, vector<Rect> &boxes) {
    Rect bestHatch = Rect(0,0,0,0);
    for (int i = 0; i < boxes.size(); i++) {
        if (classses[i].equals("hatch")) { // TODO: fix .equals if it doesnt work, change "hatch"
            Rect box = boxes[i];
            double boxArea = box.width * box.height;
            if (boxArea > bestHatch.width * bestHatch.height) {
                bestHatch = box;
            }
        }
    }

    return bestHatch;
}

Rect bestHole(vector<int> &boxClasses, vector<float> &confidences, vector<Rect> &boxes) {
    Rect bestHole = Rect(0,0,0,0);
    for (int i = 0; i < boxes.size(); i++) {
        if (classses[i].equals("hole")) { // TODO: fix .equals if it doesnt work, change "hole"
            Rect box = boxes[i];
            double boxArea = box.width * box.height;
            if (boxArea > bestHole.width * bestHole.height) {
                bestHole = box;
            }
        }
    }

    return bestHole;
}

float angleToRect(Rect target) {
    float centerX = target.x + target.width/2 - W/2;
    float angle = atan(centerX / focal_length);
    return angle;
}

double xErrorToRect(Rect target, float topToCameraDist, float bottomToCameraDist) {

    target.x -= W/2;
    target.y -= H/2;
    float topAngle = atan(target.y / focal_length);
    float bottomAngle = atan((target.y + target.height) /  focal_length);

    // y/x = tan(angle)
    float rEst1 = topToCameraDist / tan(p + topAngle);
    float rEst2 = bottomToCameraDist / tan(p + bottomAngle);
    float r = (rEst1 + rEst2) / 2.0;

    float adjustedHeight = holeHeight / r * focal_length;
    float distortion = target.width / adjustedHeight;

    return distortion;

} 

VisionResultsPackage calculate(const Mat &bgr) {

    ui64 time_began = millis_since_epoch();
    VisionResultsPackage res;

    // Create a 4D blob from a frame.
    blobFromImage(bgr, blob, 1 / 255.0, cvSize(W, H), Scalar(0, 0, 0), true, false);

    //Sets the input to the network
    net.setInput(blob);

    // Runs the forward pass to get output of the output layers
    vector<Mat> outs;
    net.forward(outs, getOutputsNames(net));

    // Remove the bounding boxes with low confidence
    vector<string> boxClasses;
    vector<float> confidences;
    vector<Rect> boxes;
    postprocess(frame, outs, boxClasses, confidences, boxes);

    Rect bestHatch = bestHatch(boxClasses, confidences, boxes);
    Rect bestHole = bestHole(boxClasses, confidences, boxes);

    res.hatchValid = false;
    res.holeValid = false;
    res.timestamp = time_began;
    if (bestHatch.width != 0) {
        res.hatchValid = true;
        res.hatchAngle = angleToRect(bestHatch);
        res.hatchDisplace = xErrorToRect(bestHatch, hatchTopToCamera, hatchBottomToCamera);
    }
    if (bestHole.width != 0) {
        res.holeValid = true;
        res.holeAngle = angleToRect(bestHole);
        res.holeDisplace = xErrorToRect(bestHole, holeTopToCamera, holeBottomToCamera);
    }

    // Put efficiency information. The function getPerfProfile returns the
    // overall time for inference(t) and the timings for each of the layers(in layersTimes)
    vector<double> layersTimes;
    double freq = getTickFrequency() / 1000;
    double t = net.getPerfProfile(layersTimes) / freq;

    return res;
}