#ifndef FR_SPLITTER_H
#define FR_SPLITTER_H

#include "frame_recipient.h"

#include <vector>

class FRSplitter : public FrameRecipient {
public:
    FRSplitter();
    ~FRSplitter();

    void initializePacket(DataPacket *dp);
    void receiveFrame(DataPacket *dp);

    void addFrameRecipient(FrameRecipient *fr);

protected:
    std::vector<FrameRecipient *> _fr;
};

#endif