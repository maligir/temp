#ifndef KFR_DEPTH_TO_POINTCLOUD_H
#define KFR_DEPTH_TO_POINTCLOUD_H

#include "kinect_frame_recipient.h"
#include <k4a/k4a.hpp>

class KFRDepthToPointcloud : public KinectFrameRecipient {
public:
    KFRDepthToPointcloud(const k4a::calibration* k4a_calibration);
    ~KFRDepthToPointcloud();

    void initializePacket(KinectPacket* kp);
    void receiveFrame(KinectPacket* kp);

protected:
    const k4a::calibration* _k4a_calibration;
    k4a::transformation k4a_transformation_;
};

#endif