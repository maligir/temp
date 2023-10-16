#ifndef PLACARD_H
#define PLACARD_H

#include "eigen3/Eigen/Dense"
#include <chrono>

struct Placard {
  const double height_ = .051;
  const double width_ = .151;
  Eigen::Vector3f position_;
  Eigen::Quaternionf orientation_;
  std::string text_;
  float conf_;
  std::chrono::microseconds obsv_time_;
  
  Placard() {}
  
  Placard (Eigen::Vector3f position,
           Eigen::Quaternionf orientation,
           std::string text,
           float conf,
           std::chrono::microseconds& obsv_time):
           position_(position),
           orientation_(orientation),
           text_(text),
           conf_(conf),
           obsv_time_(obsv_time) {}

  Placard (const Placard& other):
      height_(other.height_),
      width_(other.width_),
      position_(other.position_),
      orientation_(other.orientation_),
      text_(other.text_),
      conf_(other.conf_),
      obsv_time_(other.obsv_time_) {}
           
  bool operator==(const Placard& other) const {
    float angle =
        fabs(orientation_.angularDistance(other.orientation_));
    float dx = other.position_.x() - position_.x();
    float dy = other.position_.y() - position_.y();
    return (dx*dx+dy*dy < 0.6 && angle < 0.65);
  }
  
  void operator=(const Placard& other) {
    position_ = other.position_;
    orientation_ = other.orientation_;
    text_ = other.text_;
    conf_ = other.conf_;
    obsv_time_ = other.obsv_time_;
  }
};

#endif