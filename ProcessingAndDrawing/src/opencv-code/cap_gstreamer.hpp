#ifndef CAP_GSTREAMER_HPP
#define CAP_GSTREAMER_HPP

#include "precomp.hpp"
#include <unistd.h>
#include <string.h>
#include <gst/gst.h>
#include <gst/gstbuffer.h>
#include <gst/video/video.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/riff/riff-media.h>
#include <gst/pbutils/missing-plugins.h>


#define VERSION_NUM(major, minor, micro) (major * 1000000 + minor * 1000 + micro)
#define FULL_GST_VERSION VERSION_NUM(GST_VERSION_MAJOR, GST_VERSION_MINOR, GST_VERSION_MICRO)

#if FULL_GST_VERSION >= VERSION_NUM(0,10,32)
#include <gst/pbutils/encoding-profile.h>
//#include <gst/base/gsttypefindhelper.h>
#endif


#ifdef NDEBUG
#define CV_WARN(message)
#else
#define CV_WARN(message) fprintf(stderr, "warning: %s (%s:%d)\n", message, __FILE__, __LINE__)
#endif

#if GST_VERSION_MAJOR == 0
#error This file has been modified to only support gstreamer-1.0
#endif

#define COLOR_ELEM "autovideoconvert"
#define COLOR_ELEM_NAME COLOR_ELEM

void toFraction(double decimal, double &numerator, double &denominator);

static cv::Mutex gst_initializer_mutex;

/*!
 * \brief The CvCapture_GStreamer class
 * Use GStreamer to capture video
 */
class CvCapture_GStreamer
{
public:
    CvCapture_GStreamer() { init(); }
    ~CvCapture_GStreamer() { close(); }

    bool open( int type, const char* filename );
    bool openSplitPipeline(const char* device, int width, int height, int framerate, int bitrate, const char *ip, int port);
    void close();

    GstElement *pipeline;
    GstElement *writePipeline;
    gint width;
    gint height;
    int frameCount;

  protected:
    void init();
    bool reopen();
    bool isPipelinePlaying();
    void startPipeline();
    void stopPipeline();
    void restartPipeline();
    
    GstElement *uridecodebin;
    GstElement*   appSrc;
    GstElement*   v4l2src;
    GstElement*   color;
    GstElement*   sink;

    GstCaps*      caps;
    gint64        duration;
    double        fps;
};

#endif