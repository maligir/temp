#ifndef _FR_BGRA_TO_RGB_H
#define _FR_BGRA_TO_RGB_H

#include "frame_recipient.h"

#include <string>

class FRBGRAToRGB : public FrameRecipient {
public:
    FRBGRAToRGB(const std::string& bgra_name,
                const std::string& rgb_name);
    ~FRBGRAToRGB();
    
    void initializePacket(DataPacket *dp);
    void receiveFrame(DataPacket *dp);
private:
    const std::string _bgra_name;
    const std::string _rgb_name;
};

#endif