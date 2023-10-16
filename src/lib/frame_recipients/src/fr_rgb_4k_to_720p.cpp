#include "fr_rgb_4k_to_720p.h"
#include "data_packet.h"

#include "opencv2/highgui.hpp"

FRRGB4KTo720p::FRRGB4KTo720p() {}
FRRGB4KTo720p::~FRRGB4KTo720p() {}

void FRRGB4KTo720p::initializePacket(DataPacket *dp) {
    dp->_mats["RGB720p"] = cv::Mat(720, 1280, CV_8UC3);
    dp->_mats["Depth720p"] = cv::Mat(720, 1280, CV_32F);

    if (_fr) {
        _fr->initializePacket(dp);
    }
}

void FRRGB4KTo720p::receiveFrame(DataPacket *dp) {
    cv::resize(dp->_mats["RGB4K"], dp->_mats["RGB720p"], cv::Size(1280,720));
    if (dp->_mats.count("Depth4K")) {
        cv::resize(dp->_mats["Depth4K"],
                   dp->_mats["Depth720p"],
                   cv::Size(1280,720),
                   0,0, cv::INTER_NEAREST);
    }

    if (_fr) {
        _fr->receiveFrame(dp);
    }
}