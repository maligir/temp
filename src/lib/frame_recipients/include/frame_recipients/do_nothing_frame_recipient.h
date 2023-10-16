#ifndef DO_NOTHING_FRAME_RECIPIENT_H
#define DO_NOTHING_FRAME_RECIPIENT_H

#include "frame_recipient.h"

class DoNothingFrameRecipient : public FrameRecipient {
public:
    void initializePacket(DataPacket *dp);
    void receiveFrame(DataPacket *dp);
};

#endif