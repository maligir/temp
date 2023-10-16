#ifndef ROS_FRAME_RECIPIENT_H
#define ROS_FRAME_RECIPIENT_H

#include "frame_recipient.h"
#include "ros_packet.h"

class ROSPacket;

class ROSFrameRecipient : public FrameRecipient {
public:
  virtual ~ROSFrameRecipient() {}
  virtual void initializePacket(ROSPacket *kp) = 0;
  virtual void receiveFrame(ROSPacket *kp) = 0;

  void initializePacket(DataPacket *dp) {
    ROSPacket* rp = dynamic_cast<ROSPacket*>(dp);
    if (rp) {
        initializePacket(rp);
    } else {
      std::cerr << "WANRING: Unable to initialize data packet, "
                << " recieved DataPacket* type incompatible with type ROSDataPacket*\n";
    }
  }

  void receiveFrame(DataPacket *dp) {
    ROSPacket* rp = dynamic_cast<ROSPacket*>(dp);
    if (rp) {
        receiveFrame(rp);
    }
  }
};

#endif