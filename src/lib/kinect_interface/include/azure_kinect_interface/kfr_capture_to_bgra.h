#ifndef KFR_CAPTURE_TO_BGRA_H
#define KFR_CAPTURE_TO_BGRA_H

#include "kinect_frame_recipient.h"

class KFRCaptureToBGRA : public KinectFrameRecipient  {
public:
    KFRCaptureToBGRA();
    ~KFRCaptureToBGRA();
    
    void initializePacket(KinectPacket *kp);
    void receiveFrame(KinectPacket *kp);  
};

#endif