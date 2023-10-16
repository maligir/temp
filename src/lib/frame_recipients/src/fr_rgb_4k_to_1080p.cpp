#include "fr_rgb_4k_to_1080p.h"
#include "data_packet.h"

#include "opencv2/highgui.hpp"

FRRGB4KTo1080p::FRRGB4KTo1080p() {}
FRRGB4KTo1080p::~FRRGB4KTo1080p() {}

void FRRGB4KTo1080p::initializePacket(DataPacket *dp) {
    dp->_mats["RGB1080p"] = cv::Mat(1080, 1920, CV_8UC3);
    dp->_mats["Depth1080p"] = cv::Mat(1080, 1920, CV_32F);

    if (_fr) {
        _fr->initializePacket(dp);
    }
}

void FRRGB4KTo1080p::receiveFrame(DataPacket *dp) {
    cv::resize(dp->_mats["RGB4K"], dp->_mats["RGB1080p"], cv::Size(1920,1080));
    if (dp->_mats.count("Depth4K")) {
        cv::resize(dp->_mats["Depth4K"],
                   dp->_mats["Depth1080p"],
                   cv::Size(1920,1080),
                   0,0, cv::INTER_NEAREST);
    }

    if (_fr) {
        _fr->receiveFrame(dp);
    }
}