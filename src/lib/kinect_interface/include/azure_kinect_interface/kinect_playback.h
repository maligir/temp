#ifndef KINECT_PLAYBACK_H
#define KINECT_PLAYBACK_H

#include "device_playback.h"
#include "kinect_packet.h"
#include "kinect_frame_recipient.h"

#include <string>
#include <k4a/k4a.hpp>
#include <k4arecord/playback.h>
#include <k4arecord/playback.hpp>

class KinectFrameRecipient;

class KinectPlayback : public DevicePlayback {
public:
    KinectPlayback(std::string fname);
    ~KinectPlayback();

    bool isFinished();

    void start();
    void doOnce();

    void seekFrame(const std::chrono::microseconds& tstamp);

    k4a::calibration GetCalibration();

    DataPacket& getDataPacket() {return _pkt;}

protected:
    KinectPacket _pkt;
    k4a::playback _playback;
    std::string fname_;
    bool _isFinished;
};

#endif