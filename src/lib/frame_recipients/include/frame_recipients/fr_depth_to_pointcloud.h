#ifndef FR_DEPTH_TO_POINTCLOUD_H
#define FR_DEPTH_TO_POINTCLOUD_H

#include "frame_recipient.h"

#include <opencv2/opencv.hpp>

#include <string>

class FRDepthToPointcloud : public FrameRecipient {
public:
  FRDepthToPointcloud(const std::string& cam_name);
  ~FRDepthToPointcloud();

  void initializePacket(DataPacket* dp);
  void receiveFrame(DataPacket* dp);

private:
  std::string _cam_name;
};

#endif