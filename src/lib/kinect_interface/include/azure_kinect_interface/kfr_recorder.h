#ifndef KFR_RECORDER_H
#define KFR_RECORDER_H

#include "kinect_frame_recipient.h"

#include <k4a/k4a.hpp>
#include <k4arecord/record.hpp>

#include <string>

class KFRRecorder : public KinectFrameRecipient {
public:
    KFRRecorder(
        const char *path,
        const k4a::device &device,
        const k4a_device_configuration_t &device_configuration);
    ~KFRRecorder();

    void open(std::string filename);

    void initializePacket(KinectPacket *kp);
    void receiveFrame(KinectPacket *kp);

protected:
    k4a::record _record;
};

#endif