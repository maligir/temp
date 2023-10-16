#ifndef DATA_PACKET_H
#define DATA_PACKET_H

#include <map>
#include <string>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Core>

#include "bounding_box.h"
#include "placard.h"

class DataPacket {
public:
  DataPacket(): tframe(0) {}
  virtual ~DataPacket() {}

  std::map<int, Eigen::Matrix<double, 2, 4>> _aprilTagDetections;
  std::map<std::string, cv::Mat> _mats;
  std::map<std::string, Eigen::MatrixXf> _eigs;
  std::chrono::microseconds tframe;
  std::vector<BoundingBox> _boxes;
  std::vector<Placard> _placards;
  Eigen::Affine3f rgb_to_world;
  size_t frameID;
  size_t mapID;
};

#endif