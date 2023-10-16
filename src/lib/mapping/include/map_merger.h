#ifndef KFR_MAP_MERGER_H
#define KFR_MAP_MERGER_H

#include "System.h"

class DataPacket;

class MapMerger {
public:
    virtual Eigen::Matrix4f getTransform() = 0;
    virtual void updateTracking(DataPacket* dp) = 0;
    virtual void updateLost(DataPacket* dp) = 0;
    virtual void updateTrackingUnmerged(DataPacket* dp) = 0;
    virtual void updateLostUnmerged(DataPacket* dp) = 0;
    virtual bool isMergeReady() = 0;
};

#endif