#ifndef DEVICE_PLAYBACK_H
#define DEVICE_PLAYBACK_H

#include "do_once.h"
#include "frame_recipient.h"
#include "data_packet.h"

#include <chrono>

class DevicePlayback : public DoOnce {
public:
  DevicePlayback(): _fr(NULL) {}
  ~DevicePlayback() {}
  virtual bool isFinished() = 0;
  virtual void start() = 0;
  virtual void seekFrame(const std::chrono::microseconds& tstamp) = 0;
  virtual DataPacket& getDataPacket() = 0;

  void setFrameRecipient(FrameRecipient *fr) {
    _fr = fr;
  }

  FrameRecipient* getLastFR() {
    if (_fr) {
      return _fr->getLastFR();
    } else {
      return NULL;
    }
  }
protected:
  FrameRecipient* _fr;
};

#endif