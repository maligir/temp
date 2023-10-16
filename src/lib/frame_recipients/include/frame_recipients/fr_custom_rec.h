#ifndef FR_CUSTOM_REC_H
#define FR_CUSTOM_REC_H

#include "frame_recipient.h"

#include <string>

class FRCustomRec : public FrameRecipient {
public:
    FRCustomRec(const char *path);
    ~FRCustomRec();

    void initializePacket(DataPacket *dp);
    void receiveFrame(DataPacket *dp);

protected:
};

#endif