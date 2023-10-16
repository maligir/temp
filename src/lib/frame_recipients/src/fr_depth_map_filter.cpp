#include "fr_depth_map_filter.h"
#include "data_packet.h"

FRDepthMapFilter::FRDepthMapFilter(float dmin, float dmax, const std::string& depth_key) :
    _dmin(dmin), _dmax(dmax), _depth_key(depth_key) {}

FRDepthMapFilter::~FRDepthMapFilter() {}

void FRDepthMapFilter::initializePacket(DataPacket* dp) {
    if (_fr) {
        _fr->initializePacket(dp);
    }
}

void FRDepthMapFilter::receiveFrame(DataPacket* dp) {
    cv::Mat& depth_cv = dp->_mats[_depth_key];

    for (size_t i = 0; i < depth_cv.rows; i++) {
        for (size_t j = 0; j < depth_cv.cols; j++) {
            float& d = depth_cv.at<float>(i,j);
            if (d < _dmin || d > _dmax || std::isnan(d)) {
                d = 0;
            }
        }
    }

    if (_fr) {
        _fr->receiveFrame(dp);
    }
}