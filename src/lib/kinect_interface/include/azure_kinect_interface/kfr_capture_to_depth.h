#ifndef KFR_CAPTURE_TO_DEPTH_H
#define KFR_CAPTURE_TO_DEPTH_H

#include "kinect_frame_recipient.h"
#include <k4a/k4a.hpp>

class KFRCaptureToDepth : public KinectFrameRecipient {
public:
    KFRCaptureToDepth(const k4a::calibration* k4a_calibration);
    ~KFRCaptureToDepth();

    void initializePacket(KinectPacket *kp);
    void receiveFrame(KinectPacket *kp);

protected:
    k4a::transformation k4a_transformation_;
    const k4a::calibration* k4a_calibration_;
};
#endif