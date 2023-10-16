#include "kinect_wrapper.h"
#include "kinect_frame_recipient.h"
#include "kinect_packet.h"
#include "kfr_recorder_factory.h"

#include <k4arecord/playback.h>

#include <iostream>

using namespace std;

KinectWrapper::KinectWrapper(uint8_t deviceIndex) : _device(NULL), _fr(nullptr) {
    _config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    _config.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;
    _config.camera_fps = K4A_FRAMES_PER_SECOND_15;
    _config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    _config.color_resolution = K4A_COLOR_RESOLUTION_3072P;
    _config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
    _config.subordinate_delay_off_master_usec = 0;
    _config.synchronized_images_only = true;

    _device = k4a::device::open(deviceIndex);
    cout << deviceIndex << ": Device \"" << _device.get_serialnum() << "\"" << endl;
    _device.start_cameras(&_config);

    //start();
    _sensor_calibration = _device.get_calibration(_config.depth_mode, _config.color_resolution);
}

KinectWrapper::~KinectWrapper() {
    _device.stop_cameras();
    _device.close();
}

KFRRecorder *KinectWrapper::getKFRRecorder(KFRRecorderFactory &kfrrf) {
    return kfrrf.getRecorder(_device, _config);
}

void KinectWrapper::start() {
    if (_fr) {
        _fr->initializePacket(&_pkt);
    }
}

void KinectWrapper::doOnce() {
    _device.get_capture(&(_pkt._capture), std::chrono::milliseconds(K4A_WAIT_INFINITE));
    if (_fr) {
        _fr->receiveFrame(&_pkt);
    }
}

k4a::calibration& KinectWrapper::GetCalibration() {return _sensor_calibration;}