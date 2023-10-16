#ifndef FR_MAP_MERGING_HANDLER_H
#define FR_MAP_MERGING_HANDLER_H

#include "frame_recipient.h"
#include "System.h"
#include "map_merger.h"

class DataPacket;

class FRMapMergingHandler : public FrameRecipient {
public:
    FRMapMergingHandler(ORB_SLAM3::System& ORB_SLAM,
                        MapMerger* map_merger);
    ~FRMapMergingHandler();
    void initializePacket(DataPacket *dp);
    void receiveFrame(DataPacket *dp);

private:
    MapMerger* _map_merger;
    ORB_SLAM3::System& _ORB_SLAM;
};

#endif