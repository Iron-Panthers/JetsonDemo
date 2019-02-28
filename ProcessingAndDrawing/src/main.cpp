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
static GMainLoop* loop;
static GstElement* pipeline;

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

static bool message_cb(GstBus *bus, GstMessage *message, gpointer user_data)
{
    switch (GST_MESSAGE_TYPE(message))
    {
    case GST_MESSAGE_ERROR:
    {
        GError *err = NULL;
        gchar *name, *debug = NULL;

        name = gst_object_get_path_string(message->src);
        gst_message_parse_error(message, &err, &debug);

        g_printerr("ERROR: from element %s: %s\n", name, err->message);
        if (debug != NULL)
            g_printerr("Additional debug info:\n%s\n", debug);

        g_error_free(err);
        g_free(debug);
        g_free(name);

        g_main_loop_quit(loop);
        break;
    }
    case GST_MESSAGE_WARNING:
    {
        GError *err = NULL;
        gchar *name, *debug = NULL;

        name = gst_object_get_path_string(message->src);
        gst_message_parse_warning(message, &err, &debug);

        g_printerr("ERROR: from element %s: %s\n", name, err->message);
        if (debug != NULL)
            g_printerr("Additional debug info:\n%s\n", debug);

        g_error_free(err);
        g_free(debug);
        g_free(name);
        break;
    }
    case GST_MESSAGE_EOS:
    {
        g_print("Got EOS\n");
        g_main_loop_quit(loop);
        gst_element_set_state(pipeline, GST_STATE_NULL);
        g_main_loop_unref(loop);
        gst_object_unref(pipeline);
        exit(0);
        break;
    }
    default:
        break;
    }

    return TRUE;
}

int main(int argc, char *argv[])
{
    //call the bash script to set camera settings
    flash_good_settings();
    gst_init(&argc, &argv);

    // initialize NetworkTables
    NetworkTable::SetClientMode();
    NetworkTable::SetDSClientEnabled(false);
    NetworkTable::SetIPAddress(llvm::StringRef(netTableAddress));
    NetworkTable::Initialize();
    if (verbose) printf ("Initialized table\n");
    myNetworkTable = NetworkTable::GetTable(tableName);

    CvCapture_GStreamer mycam;
    mycam.openSplitPipeline(deviceName, width, height, framerate, bitrate, ip, port);

    if (verbose)
    {
        printf("Succesfully opened camera with dimensions: %dx%d\n", width, height);
        cout << "Data header: " << VisionResultsPackage::createCSVHeader().c_str() << endl;
    }

    pipeline = mycam.pipeline;
    loop = g_main_loop_new(NULL, FALSE);

    GstBus *bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
    gst_bus_add_signal_watch(bus);
    g_signal_connect(G_OBJECT(bus), "message", G_CALLBACK(message_cb), NULL);
    gst_object_unref(GST_OBJECT(bus));
    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    g_print("Starting loop");
    g_main_loop_run(loop);

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

