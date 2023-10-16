#ifndef FR_DEPTH_MAP_FILTER_H
#define FR_DEPTH_MAP_FILTER_H

#include "frame_recipient.h"
#include <k4a/k4a.hpp>

class FRDepthMapFilter : public FrameRecipient {
public:
    FRDepthMapFilter(float depth_min,
                     float depth_max,
                     const std::string& depth_key);
    ~FRDepthMapFilter();

    void initializePacket(DataPacket* dp);
    void receiveFrame(DataPacket* dp);

protected:
    const float _dmin, _dmax;
    const std::string _depth_key;
};

#endif