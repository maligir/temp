#include "kinect_playback.h"

using namespace k4a;

KinectPlayback::KinectPlayback(std::string fname) :
  fname_(fname),
  _isFinished(false) {
    try {
        _playback = playback::open(fname_.c_str());
    } catch (const std::exception& e) {
        printf("Failed to open recording:\n%s", e.what());
        _isFinished = true;
        return;
    }
    _playback.set_color_conversion(K4A_IMAGE_FORMAT_COLOR_BGRA32);
    uint64_t recording_length = _playback.get_recording_length().count();
    printf("Recording is %lu seconds long\n", recording_length / 1000000); 
}

KinectPlayback::~KinectPlayback() {
    _playback.close();
}

void KinectPlayback::start() {
    if (_fr) {
        _fr->initializePacket((DataPacket*)&_pkt);
    }
}

bool KinectPlayback::isFinished() {
    return _isFinished;
}

void KinectPlayback::seekFrame(const std::chrono::microseconds& tstamp) {
    _playback.seek_timestamp(tstamp, K4A_PLAYBACK_SEEK_DEVICE_TIME);
    _pkt.tframe = tstamp;
    _isFinished = false;
}

void KinectPlayback::doOnce() {
    try {
        _isFinished = !_playback.get_next_capture(&(_pkt._capture));
    } catch (const std::exception& e) {
        printf("Failed to reach end of file:\n%s", e.what());
        _isFinished = true;
        return;
    }

    if (!_isFinished) {
        if (_fr) {
            _fr->receiveFrame((DataPacket*)&_pkt);
        }
    } else {
        printf("End of file reached");
    }
}

k4a::calibration KinectPlayback::GetCalibration() {
    return _playback.get_calibration();
}