#include "kfr_release_capture.h"
#include "kinect_packet.h"

KFRReleaseCapture::KFRReleaseCapture() {}

KFRReleaseCapture::~KFRReleaseCapture() {}

void KFRReleaseCapture::initializePacket(KinectPacket* kp) {
}

void KFRReleaseCapture::receiveFrame(KinectPacket* kp) {
    kp->_capture.reset();

    if (_fr) {
        _fr->initializePacket(kp);
        _fr->receiveFrame(kp);
    }
}