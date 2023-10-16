#ifndef KINECT_PACKET_H
#define KINECT_PACKET_H

#include <k4a/k4a.hpp>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Core>

#include "data_packet.h"

#include <map>
#include <string>
#include <chrono>

class KinectPacket : public DataPacket {
public:
    KinectPacket();
    ~KinectPacket();

    k4a::capture _capture;
    std::map<std::string, k4a::image> _images;
};

#endif