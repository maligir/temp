#ifndef IMAGE_FEATURES_FEATURE_H
#define IMAGE_FEATURES_FEATURE_H

#include <opencv2/opencv.hpp>

#include "frame.h"

namespace features {

// generic feature class
template <typename Derived>
class Feature {
public:
  Feature(const Frame* frame): _frame(frame) {
    static_assert(std::is_same<Derived, decltype(*this)>::value,
                      "Derived class does not match the template argument.");
    static_assert(std::is_base_of<Feature, Derived>::value,
                      "Derived class must inherit from Feature.");
  }
  const Frame* getFrame() {return _frame;}
  virtual double getMatchDistance(const Derived* other) const = 0;
protected:
  const Frame* _frame;
};

// feature describing a point in 3D
template <typename Derived>
class XYZFeature : public Feature<XYZFeature<Derived>> {
public:
  XYZFeature(const Frame* frame) : Feature<XYZFeature<Derived>>(frame) {
    static_assert(std::is_same<Derived, decltype(*this)>::value,
                      "Derived class does not match the template argument.");
    static_assert(std::is_base_of<XYZFeature<Derived>, Derived>::value,
                      "Derived class must inherit from XYZFeature.");
  }
  virtual Eigen::Vector3d getPoint() = 0;
};

// feature describing a single pixel with unknown depth
template <typename Derived>
class PixelFeature : public Feature<PixelFeature<Derived>> {
public:
  PixelFeature(const Frame* frame, double depth_guess = 0) : 
      Feature<PixelFeature<Derived>>(frame), _depth(depth_guess) {
    static_assert(std::is_same<Derived, decltype(*this)>::value,
                      "Derived class does not match the template argument.");
    static_assert(std::is_base_of<PixelFeature<Derived>, Derived>::value,
                      "Derived class must inherit from PixelFeature.");
  }
  virtual cv::Point2d getPixel() = 0;
  double* getDepth() {return &_depth;}
protected:
  double _depth;
};

template <typename Derived>
class XYZPixelFeature : public PixelFeature<XYZPixelFeature<Derived>>,
                        public XYZFeature<XYZPixelFeature<Derived>> {
public:
  XYZPixelFeature(const Frame* frame, double depth_guess = 0):
    PixelFeature<XYZPixelFeature<Derived>>(frame, depth_guess),
    XYZFeature<XYZPixelFeature<Derived>>(frame) {
    static_assert(std::is_same<Derived, decltype(*this)>::value,
                      "Derived class does not match the template argument.");
    static_assert(std::is_base_of<XYZPixelFeature<Derived>, Derived>::value,
                      "Derived class must inherit from XYZPixelFeature.");
  }

  Eigen::Vector3d getPoint() {
    cv::Point2d pxl = this->getPixel();
    Eigen::Vector3d pnt(_depth*pxl.x(),_depth*pxl.y(), _depth);
    return self->frame->_Kinv*pnt; 
  }
};

}

#endif