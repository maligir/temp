#ifndef KFR_GET_BOUNDING_BOXES_H
#define KFR_GET_BOUNDING_BOXES_H

#include "frame_recipient.h"
#include "observer.h"
#include <string>

class FRGetBoundingBoxes : public FrameRecipient {
public:
    FRGetBoundingBoxes(Observer* observer,
                       const std::string& img_name,
                       const bool convertToRGB = false);

    void initializePacket(DataPacket *dp);
    void receiveFrame(DataPacket *dp);

protected:
    Observer* _observer;
    const std::string _img_name;
    const bool _convertToRGB;
};

#endif