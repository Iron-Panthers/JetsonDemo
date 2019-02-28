#ifndef NETTABLEMANAGER_HPP
#define NETTABLEMANAGER_HPP

#include "vision.hpp"

/*!
 * \brief The CvCapture_GStreamer class
 * Use GStreamer to capture video
 */
class NetTableManager
{
    public:
        static void pushToNetworkTables(VisionResultsPackage info);

    protected:
        NetTableManager() { init(); }
        void init();

        static shared_ptr<NetworkTable> networkTable;
        static NetTableManager* instance;
};

#endif