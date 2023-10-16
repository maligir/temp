#ifndef ROS_INTERFACE_ROSBAG_PLAYBACK_H
#define ROSBAG_PLAYBACK_H

#include "do_once.h"
#include "ros_packet.h"
#include "frame_recipient.h"
#include "device_playback.h"
#include <string>
#include <chrono>

#include <rosbag/bag.h>
#include <rosbag/view.h>

class FrameRecipient;

class ROSPlayback : public DevicePlayback {
public:
  ROSPlayback(const std::string& fname,
              const std::vector<std::string>& topics);
  ~ROSPlayback();

  bool isFinished();

  void start();
  void doOnce();

  void seekFrame(const std::chrono::microseconds& tstamp);

  DataPacket& getDataPacket() {return _pkt;}

protected:
  const std::string _fname;
  bool _isFinished;
  rosbag::Bag _bag;
  rosbag::View _view;
  const std::vector<std::string> _topics;
  ROSPacket _pkt;
};

#endif