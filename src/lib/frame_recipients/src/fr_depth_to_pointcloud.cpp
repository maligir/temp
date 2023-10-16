#include "fr_depth_to_pointcloud.h"
#include "data_packet.h"

FRDepthToPointcloud::FRDepthToPointcloud(
  const std::string& cam_name):
    _cam_name(cam_name) {}

FRDepthToPointcloud::~FRDepthToPointcloud() {}

void FRDepthToPointcloud::initializePacket(DataPacket* dp) {
  if (dp->_mats.count(_cam_name) == 0) {
    std::cerr << "Image frame " << _cam_name << " not initialized in data packet.\n";
    exit(1);
  }

  const cv::Mat& img = dp->_mats[_cam_name];
  dp->_mats["PointCloudImg"] = cv::Mat(img.rows, img.cols, CV_16SC3);
  dp->_mats["PointCloudImg4K"] = dp->_mats["PointCloudImg"];

  if (_fr) {
    _fr->initializePacket(dp);
  }
}

void FRDepthToPointcloud::receiveFrame(DataPacket* dp) {
  std::string Kinv_name = _cam_name+"_Kinv";
  if (dp->_eigs.count(Kinv_name) == 0) {
    std::cerr << "Intrinsics for camera " << _cam_name << " not found\n";
    exit(1);
  }

  const Eigen::Matrix4f& Kinv = dp->_eigs[Kinv_name].block<4,4>(0,0);

  const cv::Mat& depth = dp->_mats[_cam_name];
  cv::Mat& pc = dp->_mats["PointCloudImg"];

  for(size_t r = 0; r < depth.rows; r++) {
    for (size_t c = 0; c < depth.cols; c++) {
      float d = 1e3*depth.at<float>(r, c);
      // std::cout << c << "," << r << "," << d << std::endl;
      Eigen::Vector4f v(c,r,1,1);
      v = Kinv*v;
      v = d*v;
      // std::cout << "v: " << v.transpose() << std::endl;
      pc.at<cv::Vec3s>(r,c) = cv::Vec3s(v[0]+0.5, v[1]+0.5, v[2]+0.5);
    }
  }

  if (_fr) {
    _fr->receiveFrame(dp);
  }
}