#ifndef RFR_GET_FRAME_INFO_H
#define RFR_GET_FRAME_INFO_H

#include "ros_frame_recipient.h"

class ROSPacket;

class RFRGetFrameInfo :  public ROSFrameRecipient {
public:
  RFRGetFrameInfo(const std::string& topic,
                  const std::string& cam_name);
  
  void initializePacket(ROSPacket *rp);
  void receiveFrame(ROSPacket *rp);

private:
  bool _isInfoSet;
  const std::string _topic;
  const std::string _cam_name;
  const std::string _Kname;
  const std::string _invKname;
};

#endif