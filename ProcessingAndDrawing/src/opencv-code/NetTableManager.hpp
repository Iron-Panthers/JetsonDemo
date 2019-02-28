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
        void pushToNetworkTables(VisionResultsPackage info);
        static NetTableManager* getInstance();

    private:
        NetTableManager();
        shared_ptr<NetworkTable> networkTable;

        static NetTableManager* instance;
};

#endif