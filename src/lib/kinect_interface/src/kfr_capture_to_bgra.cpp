#include "kfr_capture_to_bgra.h"
#include "kinect_packet.h"

#include <cstring>

KFRCaptureToBGRA::KFRCaptureToBGRA() {}
KFRCaptureToBGRA::~KFRCaptureToBGRA() {}

void KFRCaptureToBGRA::initializePacket(KinectPacket *kp) {
    if (_fr) {
        _fr->initializePacket(kp);
    }
}

void KFRCaptureToBGRA::receiveFrame(KinectPacket *kp) {
    k4a::image image = kp->_capture.get_color_image();
    uint8_t *buffer = image.get_buffer();

    cv::Mat wrappedk4aimage {
        image.get_height_pixels(),
        image.get_width_pixels(),
        CV_8UC4,
        image.get_buffer(),
        (size_t)image.get_stride_bytes()
    };

    kp->_mats["BGRA4K"] = wrappedk4aimage;

    if (_fr) {
        _fr->receiveFrame(kp);
    }
}