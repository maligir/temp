#ifndef KFR_RELEASE_CAPTURE_H
#define KFR_RELEASE_CAPTURE_H

#include "kinect_frame_recipient.h"

class KFRReleaseCapture : public KinectFrameRecipient {
    public:
    KFRReleaseCapture();
    ~KFRReleaseCapture();
    
    void initializePacket(KinectPacket *kp);
    void receiveFrame(KinectPacket *kp);

};

#endif