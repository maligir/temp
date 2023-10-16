#ifndef RFR_SYNC_IMAGE_FRAMES_H
#define RFR_SYNC_IMAGE_FRAMES_H

#include "ros_frame_recipient.h"
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

typedef std::deque<sensor_msgs::ImageConstPtr> ImageQueue;

struct ImageRes {
  size_t first;
  size_t second;
  size_t third;
  uint8_t bytes;

  ImageRes() {}

  ImageRes(size_t f, size_t s, size_t t, uint8_t b):
      first(f),
      second(s),
      third(t),
      bytes(b) {}

  size_t size() {
    return first*second*third*bytes;
  }
};

class RFRSyncImageFrames : public ROSFrameRecipient {
public:
  RFRSyncImageFrames(const std::string& img1_topic,
                     const std::string& img2_topic,
                     const std::string& cam1_name,
                     const std::string& cam2_name,
                     const ImageRes& res1,
                     const ImageRes& res2,
                     uint8_t cv_format1,
                     uint8_t cv_format2);
  
  void initializePacket(ROSPacket *rp);
  void receiveFrame(ROSPacket *rp);

private:
  ImageQueue _q1;
  ImageQueue _q2;
  ImageRes _res1;
  ImageRes _res2;
  const std::string _img1_topic;
  const std::string _img2_topic;
  const std::string _cam1_name;
  const std::string _cam2_name;
  const uint8_t _cv_format1;
  const uint8_t _cv_format2;

  bool foundImageSync();
  bool searchQueueForTimeSync(double time, ImageQueue& q);
};

#endif