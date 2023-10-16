#ifndef _FR_RGB_4K_TO_1080P_H
#define _FR_RGB_4K_TO_1080P_H

#include "frame_recipient.h"

class FRRGB4KTo1080p : public FrameRecipient {
public:
    FRRGB4KTo1080p();
    ~FRRGB4KTo1080p();
    
    void initializePacket(DataPacket *dp);
    void receiveFrame(DataPacket *dp);
};

#endif