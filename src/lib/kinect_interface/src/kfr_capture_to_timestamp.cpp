#include "kfr_capture_to_timestamp.h"
#include "kinect_packet.h"

KFRCaptureToTimeStamp::KFRCaptureToTimeStamp() {}
KFRCaptureToTimeStamp::~KFRCaptureToTimeStamp() {}

void KFRCaptureToTimeStamp::initializePacket(KinectPacket* kp) {
    if (_fr) {
        _fr->initializePacket(kp);
    }
}

void KFRCaptureToTimeStamp::receiveFrame(KinectPacket* kp) {
    const k4a::image& im = kp->_capture.get_color_image();
    kp->tframe = im.get_device_timestamp();

    if (_fr) {
        _fr->receiveFrame(kp);
    }
}