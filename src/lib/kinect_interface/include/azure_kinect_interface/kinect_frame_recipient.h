#ifndef KINECT_FRAME_RECIPIENT_H
#define KINECT_FRAME_RECIPIENT_H

#include "frame_recipient.h"
#include "kinect_packet.h"

class KinectPacket;

class KinectFrameRecipient : public FrameRecipient {
public:
  virtual ~KinectFrameRecipient() {}
  virtual void initializePacket(KinectPacket *kp) = 0;
  virtual void receiveFrame(KinectPacket *kp) = 0;

  void initializePacket(DataPacket *dp) {
    KinectPacket* kp = dynamic_cast<KinectPacket*>(dp);
    if (kp) {
      initializePacket(kp);
    } else {
      std::cerr << "WANRING: Unable to initialize data packet, "
                << " recieved DataPacket* type incompatible with type KinectDataPacket*\n";
    }
  }

  void receiveFrame(DataPacket *dp) {
    KinectPacket* kp = dynamic_cast<KinectPacket*>(dp);
    if (kp) {
      receiveFrame(kp);
    }
  }
};
#endif