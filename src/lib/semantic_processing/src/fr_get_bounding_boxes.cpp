#include "fr_get_bounding_boxes.h"
#include "data_packet.h"

FRGetBoundingBoxes::FRGetBoundingBoxes(Observer* observer,
                                       const std::string& img_name,
                                       const bool convertToRGB):
    _observer(observer)
    , _img_name(img_name)
    , _convertToRGB(convertToRGB) {}

void FRGetBoundingBoxes::initializePacket(DataPacket* dp) {
    if (_fr) {
        _fr->initializePacket(dp);
    }
}

void FRGetBoundingBoxes::receiveFrame(DataPacket* dp) {
    dp->_boxes.clear();
    cv::Mat input;
    if (!_convertToRGB) {
        input = dp->_mats[_img_name];
    } else {
        cv::cvtColor(dp->_mats[_img_name], input, cv::COLOR_BGR2RGB);
    }

    _observer->getDetections(input, dp->_boxes);
    if (_fr) {
        _fr->receiveFrame(dp);
    }
}