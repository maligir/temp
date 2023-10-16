#ifndef FR_APRIL_TAG_POSE_H
#define FR_APRIL_TAG_POSE_H

#include <cstdlib>
#include <fstream>
#include <Eigen/Dense>

#include "frame_recipient.h"

#include "vision_geometry/CameraIntrinsics.h"
#include "vision_geometry/HomographyShortcuts.h"
#include "vision_geometry/HomCartShortcuts.h"
#include "vision_geometry/RigidTrans.h"
#include "vision_geometry/DistortionModel.h"
#include "vision_geometry/TransformShortcuts.h"

using namespace std;

class FRAprilTagPose : public FrameRecipient {
public:
    FRAprilTagPose();
    void initializePacket(DataPacket *dp);
    
    void receiveFrame(DataPacket *dp);
        
    void saveTheNextRT(string filename);

protected:
    CameraIntrinsics _intrinsics;
    DistortionModel _distortionModel;

    std::map<int, Eigen::MatrixXd> _aprilTagRTMats;
};

#endif