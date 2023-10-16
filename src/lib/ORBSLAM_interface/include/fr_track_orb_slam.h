#ifndef FR_TRACK_ORB_SLAM_H
#define FR_TRACK_ORB_SLAM_H

#include "frame_recipient.h"
#include "System.h"

#include <string>

class FRTrackORBSLAM : public FrameRecipient {
public:
    FRTrackORBSLAM(ORB_SLAM3::System& SLAM,
                   const std::string& rgb_name,
                   const std::string& depth_name);
    void initializePacket(DataPacket *dp);
    void receiveFrame(DataPacket *dp);

protected:
    ORB_SLAM3::System& _SLAM;
    const std::string _rgb_name;
    const std::string _depth_name;
};

#endif