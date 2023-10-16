#ifndef IMAGE_FEATURES_FRAME_H
#define IMAGE_FEATURES_FRAME_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <map>
#include <Eigen/Core>

namespace features {

class Frame {
public:
  Frame();

  double* getParams() {return _params;}
  cv::Mat getRGB() const {return _rgb_img;}
  cv::Mat getGrey() const {return _grey_img;}
  cv::Mat getDepth() const {return _depth_img;}
  size_t getID() const {return _id;}
  bool isDepthValid(cv::Point p, uint16_t& depth) const {
    depth = _depth_img.at<uint16_t>(p.y, p.x);
    return (_dmin < depth) && (depth < _dmax);
  }

  const Eigen::Matrix3d _K;
  const Eigen::Matrix3d _Kinv;

private:
  // 3 position, 4 orientation
  double _params[7];
  const size_t _id;

  cv::Mat _rgb_img;
  cv::Mat _grey_img;
  cv::Mat _depth_img;

  const uint16_t _dmin;
  const uint16_t _dmax;
};
}

#endif