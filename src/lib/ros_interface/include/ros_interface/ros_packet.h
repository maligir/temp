#ifndef ROS_PACKET_H
#define ROS_PACKET_H

#include "data_packet.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>

class ROSPacket : public DataPacket {
public:
  ROSPacket() {}
  ~ROSPacket() {}

  rosbag::View::iterator _msg;
};

#endif