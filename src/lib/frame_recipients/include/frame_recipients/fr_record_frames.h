#ifndef FR_GET_FRAME_INFO_H
#define FR_GET_FRAME_INFO_H

#include "frame_recipient.h"

#include <string>
#include <boost/filesystem.hpp>

class FRRecordFrames : public FrameRecipient {
public:
  FRRecordFrames(const std::string& rgb,
                  const std::string& depth,
                  const std::string& save_dir);
  ~FRRecordFrames();
  
  void initializePacket(DataPacket *dp);
  void receiveFrame(DataPacket *dp);

private:
  const std::string _rgb;
  const std::string _depth;
  const std::string _save_dir;
  size_t _count;
  std::ofstream _writer;
};

#endif