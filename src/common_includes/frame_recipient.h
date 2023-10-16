#ifndef FRAME_RECIPIENT_H
#define FRAME_RECIPIENT_H

#include <cstddef>

class DataPacket;

class FrameRecipient {
public:
  FrameRecipient(): _fr(NULL) {}
  virtual ~FrameRecipient() {}
  virtual void initializePacket(DataPacket *dp) = 0;
  virtual void receiveFrame(DataPacket *dp) = 0;

  void setFrameRecipient(FrameRecipient *fr) {
    _fr = fr;
  }

  FrameRecipient* getLastFR() {
    if (_fr) {
      return _fr->getLastFR();
    } else {
      return this;
    }
  }

  FrameRecipient* getFrameRecipient() {return _fr;}

  void deleteRecipients() {
    if (_fr) {
      _fr->deleteRecipients();
    }
    this->~FrameRecipient();
  }

protected:
  FrameRecipient *_fr;
};

#endif