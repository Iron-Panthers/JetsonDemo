/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                        Intel License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2008, 2011, Nils Hasler, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of Intel Corporation may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

/*!
 * \file cap_gstreamer.cpp
 * \author Nils Hasler <hasler@mpi-inf.mpg.de>
 *         Max-Planck-Institut Informatik
 * \author Dirk Van Haerenborgh <vhdirk@gmail.com>
 *
 * \brief Use GStreamer to read/write video
 */

#include "cap_gstreamer.hpp"
#include <iostream>
#include "NetTableManager.hpp"
using namespace std;
using namespace cv;

/*!
* \brief The gst_initializer class
* Initializes gstreamer once in the whole process
*/
class gst_initializer
{
  public:
    static void init()
    {
        gst_initializer_mutex.lock();
        static gst_initializer init;
        gst_initializer_mutex.unlock();
    }

  private:
    gst_initializer()
    {
        gst_init(NULL, NULL);
        //        gst_debug_set_active(1);
        //        gst_debug_set_colored(1);
        //        gst_debug_set_default_threshold(GST_LEVEL_INFO);
    }
};

/*!
* \brief CvCapture_GStreamer::init
* inits the class
*/
void CvCapture_GStreamer::init()
{
    pipeline = NULL;
    uridecodebin = NULL;
    v4l2src = NULL;
    color = NULL;
    sink = NULL;

    caps = NULL;
    duration = -1;
    width = -1;
    height = -1;
    fps = -1;
    frameCount = 0;
	num_frames = 0;
}

/*!
* \brief CvCapture_GStreamer::close
* Closes the pipeline and destroys all instances
*/
void CvCapture_GStreamer::close()
{
    if (isPipelinePlaying())
        this->stopPipeline();

    if (this->pipeline)
    {
        gst_element_set_state(GST_ELEMENT(pipeline), GST_STATE_NULL);
        gst_object_unref(GST_OBJECT(pipeline));
        pipeline = NULL;
    }

	if(this->writePipe) {
		if (gst_app_src_end_of_stream(GST_APP_SRC(this->source)) != GST_FLOW_OK) {
            CV_WARN("Cannot send EOS to GStreamer pipeline\n");
            return;
        }

        gst_element_set_state(GST_ELEMENT(this->writePipe), GST_STATE_NULL);
        gst_object_unref(GST_OBJECT(this->writePipe));
        pipeline = NULL;
    }

    duration = -1;
    width = -1;
    height = -1;
    fps = -1;
}

/*!
* \brief CvCapture_GStreamer::isPipelinePlaying
* \return if the pipeline is currently playing.
*/
bool CvCapture_GStreamer::isPipelinePlaying()
{
    GstState current, pending;
    GstClockTime timeout = 5 * GST_SECOND;
    if (!GST_IS_ELEMENT(pipeline))
    {
        return false;
    }

    GstStateChangeReturn ret = gst_element_get_state(GST_ELEMENT(pipeline), &current, &pending, timeout);
    if (!ret)
    {
        //fprintf(stderr, "GStreamer: unable to query pipeline state\n");
        return false;
    }

    return current == GST_STATE_PLAYING;
}

/*!
* \brief CvCapture_GStreamer::startPipeline
* Start the pipeline by setting it to the playing state
*/
void CvCapture_GStreamer::startPipeline()
{
    CV_FUNCNAME("icvStartPipeline");

    //fprintf(stderr, "relinked, pausing\n");
    cout << "starting pipeline" << endl;
    GstStateChangeReturn status = gst_element_set_state(GST_ELEMENT(pipeline), GST_STATE_PLAYING);
    if (status == GST_STATE_CHANGE_ASYNC)
    {
        // wait for status update
        status = gst_element_get_state(pipeline, NULL, NULL, GST_CLOCK_TIME_NONE);
    }
    if (status == GST_STATE_CHANGE_FAILURE)
    {
        gst_object_unref(pipeline);
        pipeline = NULL;
        printf("GStreamer: unable to start pipeline\n");
        return;
    }
    cout << "pipeline started" << endl;

    cout << "starting write pipeline" << endl;
    GstStateChangeReturn writeStatus = gst_element_set_state(GST_ELEMENT(writePipe), GST_STATE_PLAYING);
    if (writeStatus == GST_STATE_CHANGE_ASYNC) {
        // wait for status update
        writeStatus = gst_element_get_state(writePipe, NULL, NULL, GST_CLOCK_TIME_NONE);
    }
    if (writeStatus == GST_STATE_CHANGE_FAILURE) {
        gst_object_unref(writePipe);
        writePipe = NULL;
        printf("GStreamer: unable to start pipeline\n");
        return;
    }
    cout << "pipeline started" << endl;

    // printf("state now playing\n");
}

/*!
* \brief CvCapture_GStreamer::stopPipeline
* Stop the pipeline by setting it to NULL
*/
void CvCapture_GStreamer::stopPipeline()
{
    CV_FUNCNAME("icvStopPipeline");

    //fprintf(stderr, "restarting pipeline, going to ready\n");
    if (gst_element_set_state(GST_ELEMENT(pipeline), GST_STATE_NULL) ==
        GST_STATE_CHANGE_FAILURE)
    {
        printf("GStreamer: unable to stop pipeline\n");
        gst_object_unref(pipeline);
        pipeline = NULL;
        return;
    }
}

/*!
* \brief CvCapture_GStreamer::restartPipeline
* Restart the pipeline
*/
void CvCapture_GStreamer::restartPipeline()
{
    this->stopPipeline();
    this->startPipeline();
}

static GstFlowReturn new_sample(GstElement *sink, CvCapture_GStreamer* obj)
{
    /* Retrieve the buffer */
    GstSample *sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
    if (sample)
    {
        obj->frameCount++;
        // cout << obj->frameCount << endl;
        
        GstBuffer *buffer = gst_sample_get_buffer(sample);
        if (!buffer) {
            cout << "no buffer" << endl;
            return GST_FLOW_ERROR;
        }

        cv::Mat processedImage;
        GstMapInfo info;
        gst_buffer_map(buffer, &info, (GstMapFlags)GST_MAP_READ);
        cv::Mat frameMat = cv::Mat(cv::Size(obj->width, obj->height), CV_8UC3, (char *)info.data);
        VisionResultsPackage res = calculate(frameMat, processedImage);
        // NetTableManager::getInstance()->pushToNetworkTables(res);
        IplImage outImage = (IplImage)processedImage;
		printf("WRITING FRAME\n");
        obj->writeFrame(&outImage);

        gst_buffer_unmap(buffer, &info);
        gst_sample_unref(sample);

        return GST_FLOW_OK;
    } else {
        cout << "GRAB FAIL" << endl;
    }

    return GST_FLOW_ERROR;    
}

bool CvCapture_GStreamer::writeFrame(const IplImage *image) {
    GstClockTime duration, timestamp;
    GstFlowReturn ret;
    int size;

	if (image->nChannels != 3 || image->depth != IPL_DEPTH_8U) {
		printf("cvWriteFrame() needs images with depth = IPL_DEPTH_8U and nChannels = 3.");
	}

    size = image->imageSize;
    duration = ((double)1 / this->fps) * GST_SECOND;
    timestamp = num_frames * duration;

    //gst_app_src_push_buffer takes ownership of the buffer, so we need to supply it a copy
    GstBuffer *buffer = gst_buffer_new_allocate(NULL, size, NULL);
    GstMapInfo info;
    gst_buffer_map(buffer, &info, (GstMapFlags)GST_MAP_READ);
	printf("ABOUT TO MEMCOPY \n");
    memcpy(info.data, (guint8 *)image->imageData, size);
	printf("DID MEMCOPY \n");
    gst_buffer_unmap(buffer, &info);
    GST_BUFFER_DURATION(buffer) = duration;
    GST_BUFFER_PTS(buffer) = timestamp;
    GST_BUFFER_DTS(buffer) = timestamp;

    //set the current number in the frame
    GST_BUFFER_OFFSET(buffer) = num_frames;

	printf("ABOUT TO GET SRC BUFFER\n");
    ret = gst_app_src_push_buffer(GST_APP_SRC(source), buffer);

	printf("DID FINAL WRITE STEP \n");

    if (ret != GST_FLOW_OK) {
        CV_WARN("Error pushing buffer to GStreamer pipeline");
        return false;
    }

    //GST_DEBUG_BIN_TO_DOT_FILE(GST_BIN(pipeline), GST_DEBUG_GRAPH_SHOW_ALL, "pipeline");

    ++num_frames;

    return true;
}

bool CvCapture_GStreamer::openSplitPipeline(const char *device, int width, int height, int framerate, int bitrate, const char *ip, int port, int cvPort)
{

    this->width = width;
    this->height = height;
	this->fps = framerate;

    /* Create the elements */
    GstElement *pipeline = gst_pipeline_new("cv_pipeline");

    GstElement *camSource = gst_element_factory_make("v4l2src", "cv_src");
    g_object_set(camSource, "device", device, NULL);
    GstElement *xraw = gst_element_factory_make("capsfilter", "xraw");
    GstCaps *xrawCap = gst_caps_new_simple("video/x-raw",
                                           "width", G_TYPE_INT, width,
                                           "height", G_TYPE_INT, height,
                                           "framerate", GST_TYPE_FRACTION, framerate, 1,
                                           NULL);
    g_object_set(xraw, "caps", xrawCap, NULL);
    gst_caps_unref(xrawCap);
    GstElement *tee = gst_element_factory_make("tee", "t");

    GstElement *queue_cv = gst_element_factory_make("queue", NULL);
    GstElement *cv_convert = gst_element_factory_make("videoconvert", NULL);
    GstElement *cv_xraw = gst_element_factory_make("capsfilter", "cv_xraw");
    GstCaps *cv_xrawCap = gst_caps_new_simple("video/x-raw",
                                           "width", G_TYPE_INT, width,
                                           "height", G_TYPE_INT, height,
                                           "framerate", GST_TYPE_FRACTION, framerate, 1,
                                           "format", G_TYPE_STRING, "BGR",
                                           NULL);
    g_object_set(cv_xraw, "caps", cv_xrawCap, NULL);
    gst_caps_unref(cv_xrawCap);
    GstElement *sink_cv = gst_element_factory_make("appsink", NULL);
    this->sink = sink_cv;
    GstAppSink *sank = GST_APP_SINK(gst_object_ref(sink_cv));
    gst_app_sink_set_emit_signals(sank, true);
    gst_app_sink_set_max_buffers(sank, 1);
    gst_app_sink_set_drop(sank, true);
    g_signal_connect(sank, "new-sample", G_CALLBACK(new_sample), this);

    GstElement *queue_udp = gst_element_factory_make("queue", NULL);
    GstElement *enc_udp = gst_element_factory_make("x264enc", NULL);
    g_object_set(enc_udp, "tune", 4, "bitrate", bitrate, "speed-preset", 1, NULL);
    GstElement *rtp_udp = gst_element_factory_make("rtph264pay", NULL);
    GstElement *sink_udp = gst_element_factory_make("udpsink", NULL);
    g_object_set(sink_udp, "host", ip, "port", port, NULL);

    if (!camSource || !xraw || !tee || !queue_cv || !cv_convert || !cv_xraw || !sink_cv || !queue_udp || !enc_udp || !rtp_udp || !sink_udp)
    {
        cout << "didnt create" << endl;
    }

    gst_bin_add_many(GST_BIN(pipeline), camSource, xraw, tee, queue_cv, cv_convert, cv_xraw, sink_cv, queue_udp, enc_udp, rtp_udp, sink_udp, NULL);
    if (!gst_element_link_many(camSource, xraw, tee, NULL)) {
        cout << "didnt link start" << endl;
        return false;
    }
    if (!gst_element_link_many(tee, queue_udp, enc_udp, rtp_udp, sink_udp, NULL))
    {
        cout << "didnt link udp" << endl;
        return false;
    }
    if (!gst_element_link_many(tee, queue_cv, cv_convert, cv_xraw, sink_cv, NULL))
    {
        cout << "didnt link cv" << endl;
        return false;
    }

    this->v4l2src = camSource;
    this->pipeline = pipeline;
    printf("set pipeline\n");

    /* Create the elements */
    GstElement *writePipe = gst_parse_launch("appsrc name=app_src block=true ! video/x-raw,width=640,height=480,framerate=30/1 ! x264enc speed-preset=1 tune=zerolatency bitrate=1024 ! rtph264pay ! udpsink host=10.50.26.5 port=5803", NULL);
    GstElement* writeSource = gst_bin_get_by_name(GST_BIN(writePipe), "app_src");
    GstCaps *source_caps = gst_caps_new_simple("video/x-raw",
                                              "width", G_TYPE_INT, width,
                                              "height", G_TYPE_INT, height,
                                              "framerate", GST_TYPE_FRACTION, framerate, 1,
                                              "format", G_TYPE_STRING, "BGR",
                                              NULL);
    gst_app_src_set_caps(GST_APP_SRC(writeSource), source_caps);

    this->writePipe = writePipe;
    this->source = writeSource;
    printf("set write pipeline\n");

    return true;
}