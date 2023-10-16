#ifndef KFR_DEPTH_TO_POINTCLOUD_GPU_H
#define KFR_DEPTH_TO_POINTCLOUD_GPU_H

#include "kinect_frame_recipient.h"
#include "gpudepthtopointcloudconverter.h"
#include <k4a/k4a.hpp>

class KFRDepthToPointcloudGPU : public KinectFrameRecipient {
public: 
    KFRDepthToPointcloudGPU(const k4a::calibration& k4a_calibration,
                            k4a_calibration_type_t calibrationType);
    ~KFRDepthToPointcloudGPU();

    void setKinectFrameRecipient(KinectFrameRecipient* kfr);
    void initializePacket(KinectPacket* kp);
    void receiveFrame(KinectPacket* kp);

protected:
    KinectFrameRecipient *_kfr;
    k4aviewer::GpuDepthToPointCloudConverter _converter;
};

#endif