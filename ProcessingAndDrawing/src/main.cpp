// gst-launch-1.0 -vvv v4l2src device=/dev/video0 ! "video/x-raw,width=640,height=480,framerate=30/1" ! x264enc speed-preset=1 tune=zerolatency bitrate=1024 ! rtph264pay ! udpsink host=10.50.26.5 port=5004
// gst-launch-1.0 -vvv v4l2src device=/dev/video1 ! "video/x-raw,width=640,height=480,framerate=30/1" ! x264enc speed-preset=1 tune=zerolatency bitrate=1024 ! rtph264pay ! udpsink host=10.50.26.5 port=5005

#include "helper.hpp"
#include "gst_pipeline.hpp"
#include "vision.hpp"
#include "stdio.h"

using namespace std;

shared_ptr<NetworkTable> myNetworkTable; //our networktable for reading/writing
string netTableAddress = "192.168.1.34"; //address of the rio

//useful for testing OpenCV drawing to see you can modify an image
void fillCircle (cv::Mat img, int rad, cv::Point center);
void pushToNetworkTables (VisionResultsPackage info);

//camera parameters
int device = 1;
const char* deviceName = "/dev/video1";
int width = 640;
int height = 480;
int framerate = 30;

//network parameters
int bitrate = 1024;
int port = 5005; //destination port for raw image
const char* ip = "10.50.26.5"; //destination ip

string tableName = "CVResultsTable";

bool verbose = false;

void flash_good_settings() {
    char setting_script[100];
    sprintf (setting_script, "bash good_settings.sh %d", device);
    system (setting_script);
}

void flash_bad_settings() {
    char setting_script[100];
    sprintf (setting_script, "bash bad_settings.sh %d", device);
    system (setting_script);
}

int main(int argc, char *argv[])
{
    //call the bash script to set camera settings
    flash_good_settings();
    gst_init(&argc, &argv);

    initialize NetworkTables
    NetworkTable::SetClientMode();
    NetworkTable::SetDSClientEnabled(false);
    NetworkTable::SetIPAddress(llvm::StringRef(netTableAddress));
    NetworkTable::Initialize();
    if (verbose) printf ("Initialized table\n");
    myNetworkTable = NetworkTable::GetTable(tableName);

    CvCapture_GStreamer mycam;
    mycam.openSplitPipeline(deviceName, width, height, framerate, bitrate, ip, port);
    gst_element_set_state(mycam.pipeline, GST_STATE_PLAYING);

    if (verbose) {
        printf ("Succesfully opened camera with dimensions: %dx%d\n",
            width, height);
    }

    //initialize raw & processed image matrices
    cv::Mat cameraFrame, processedImage;

    if (verbose) {
        cout << "Data header: " <<  VisionResultsPackage::createCSVHeader().c_str() << endl;
    }

    //take each frame from the pipeline
    for (long long frame = 0; ; frame++) {
        //have to alternate from bad settings to good settings on some cameras
        //because of weird firmware issues, sometimes the flash doesn't stick 
        //otherwise
        if (frame < 10) {
            flash_bad_settings();
        }
        else if (frame == 50) {
            flash_good_settings();
        }

        bool success = mycam.grabFrame();

        if (verbose) printf ("frame #%lld\n", frame);

        if (success) {
            const IplImage *img = mycam.retrieveFrame(0); //store frame in IplImage
            cameraFrame = cv::cvarrToMat (img); //convert IplImage to cv::Mat

            // processedImage = cameraFrame;
                
            // // process the image, put the information into network tables
            VisionResultsPackage info = calculate(cameraFrame, processedImage);
            cout << "PROCESSED MAT" << endl;

            // // pushToNetworkTables (info);
          
            // //pass the results back out
            // IplImage outImage = (IplImage) processedImage;
            // printf ("results string: %s\n", info.createCSVLine().c_str());
            // if (verbose) {
            //     printf ("Out image stats: (depth %d), (nchannels %d)\n", 
            //         outImage.depth, outImage.nChannels);
            // }
        } else {
            cout << "frame failure" << endl;
        }

        //delay for 10 millisecondss
        usleep (10);
    }

    mycam.close();
    return 0;
}

void fillCircle (cv::Mat img, int rad, cv::Point center) {
    int thickness = -1;
    int lineType = 8;
    cv::circle (img, center, rad, cv::Scalar(0, 0, 255), thickness, lineType);
}

void pushToNetworkTables (VisionResultsPackage info) {
    myNetworkTable -> PutString ("VisionResults", info.createCSVLine());
    myNetworkTable -> PutString ("VisionResultsHeader", info.createCSVHeader());
    myNetworkTable -> PutNumber ("Sample Hue", info.sampleHue);
    myNetworkTable -> PutNumber ("Sample Sat", info.sampleSat);
    myNetworkTable -> PutNumber ("Sample Val", info.sampleVal);
    myNetworkTable -> Flush();
}

