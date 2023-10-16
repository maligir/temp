#ifndef INDEX_READER_PLAYBACK_H
#define INDEX_READER_PLAYBACK_H

#include "device_playback.h"

#include <fstream>
#include <iostream>

class IndexReaderPlayback : public DevicePlayback {
public:
  IndexReaderPlayback(const std::string& index);
  ~IndexReaderPlayback();

  bool isFinished();
  void start();
  void seekFrame(const std::chrono::microseconds& tstamp);
  DataPacket& getDataPacket();

  void doOnce();

private:
  const std::string _index;
  std::string _rgb;
  std::string _depth;
  std::ifstream _reader;
  DataPacket _pkt;
};

#endif