#ifndef _FR_RGB_4K_TO_720P_H
#define _FR_RGB_4K_TO_720P_H

#include "frame_recipient.h"

class FRRGB4KTo720p : public FrameRecipient {
public:
    FRRGB4KTo720p();
    ~FRRGB4KTo720p();
    
    void initializePacket(DataPacket *dp);
    void receiveFrame(DataPacket *dp);
};

#endif