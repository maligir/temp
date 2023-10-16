#ifndef FR_SAVE_PACKET_H
#define FR_SAVE_PACKET_H

#include "frame_recipient.h"

#include <vector>

class FRSavePacket : public FrameRecipient {
public:
    FRSavePacket(std::vector<DataPacket*>& packets);
    ~FRSavePacket();
    void initializePacket(DataPacket *dp);
    void receiveFrame(DataPacket *dp);

protected:
    std::vector<DataPacket*>& _packets;
};

#endif