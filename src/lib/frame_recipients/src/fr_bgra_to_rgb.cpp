#include "fr_bgra_to_rgb.h"
#include "data_packet.h"

FRBGRAToRGB::FRBGRAToRGB(const std::string& bgra_name,
                        const std::string& rgb_name):
    _bgra_name(bgra_name),
    _rgb_name(rgb_name) {}
FRBGRAToRGB::~FRBGRAToRGB() {}

void FRBGRAToRGB::initializePacket(DataPacket *dp) {
    dp->_mats[_rgb_name] = cv::Mat(3072, 4096, CV_8UC3);
    if (_fr) {
        _fr->initializePacket(dp);
    }
}

void FRBGRAToRGB::receiveFrame(DataPacket *dp) {
    cv::cvtColor(dp->_mats[_bgra_name], dp->_mats[_rgb_name], cv::COLOR_BGRA2RGB);
    
    if (_fr) {
        _fr->receiveFrame(dp);
    }
}