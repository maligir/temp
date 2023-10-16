#include "kfr_recorder.h"
#include "kinect_packet.h"

KFRRecorder::KFRRecorder(
    const char *path,
    const k4a::device &device,
    const k4a_device_configuration_t &device_configuration) {
    _record = k4a::record::create(path, device, device_configuration);
    _record.write_header();
    //_record.add_imu_track();
}

KFRRecorder::~KFRRecorder() {
    _record.flush();
    _record.close();
}

void KFRRecorder::initializePacket(KinectPacket *kp) {

}

void KFRRecorder::receiveFrame(KinectPacket *kp) {
    _record.write_capture(kp->_capture);
}