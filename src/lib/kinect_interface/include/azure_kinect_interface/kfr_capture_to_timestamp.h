#ifndef _KFR_CAPTURE_TO_TIMESTAMP_H
#define _KFR_CAPTURE_TO_TIMESTAMP_H

#include "kinect_frame_recipient.h"

class KFRCaptureToTimeStamp : public KinectFrameRecipient {
public:
    KFRCaptureToTimeStamp();
    ~KFRCaptureToTimeStamp();
    
    void initializePacket(KinectPacket *kp);
    void receiveFrame(KinectPacket *kp);

protected:
    double _last_tstamp;
};

#endif