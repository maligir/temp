#ifndef SIMPLE_POSE_H
#define SIMPLE_POSE_H

#include <Eigen/Dense>

class SimplePose {
public:
  SimplePose(const size_t& ID):
      _ID(ID) {
    _params = new double[7];
    _params[0] = 1;
    for (uint8_t i = 1; i < 7; i++) {
      _params[i] = 0;
    }
  }

  Eigen::Vector3d getPosition() {return Eigen::Vector3d(_params[4],_params[5],_params[6]);}
  Eigen::Quaterniond getOrientation() {return Eigen::Quaterniond(_params[0], _params[1], _params[2], _params[3]);}
  size_t getID() {return _ID;}
  double* getParams() {return _params;}

  bool operator==(SimplePose& other) {
    return _ID == other.getID();
  }

protected:
  const size_t _ID;
  double* _params;
};

#endif