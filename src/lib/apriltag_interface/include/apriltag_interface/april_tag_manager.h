#ifndef APRIL_TAG_MANAGER_H
#define APRIL_TAG_MANAGER_H

#include "simple_pose.h"
#include "bimap.h"

#include <apriltag/apriltag.h>
#include <sophus/se3.hpp>

#include <ceres/autodiff_cost_function.h>
#include <ceres/types.h>
#include <ceres/ceres.h>

typedef SimplePose AprilFrame;
typedef SimplePose AprilTag;

typedef Eigen::Matrix<double, 3, 4> Cornersd;

struct AprilTagCostFunctor {
  const Eigen::Vector2d _topLeft;
  const Eigen::Vector2d _topRight;
  const Eigen::Vector2d _bottomLeft;
  const Eigen::Vector2d _bottomRight;
  const double _width;
  const double _height;

  AprilTagCostFunctor(const Eigen::Vector2d& topLeft,
                      const Eigen::Vector2d& topRight,
                      const Eigen::Vector2d& bottomLeft,
                      const Eigen::Vector2d& bottomRight,
                      const double width,
                      const double height):
      _topLeft(topLeft)
      , _topRight(topRight)
      , _bottomLeft(bottomLeft)
      , _bottomRight(bottomRight)
      , _width(width)
      , _height(height) {}
  
  static ceres::CostFunction* Create(const Eigen::Vector2d& topLeft,
                              const Eigen::Vector2d& topRight,
                              const Eigen::Vector2d& bottomLeft,
                              const Eigen::Vector2d& bottomRight,
                              const double width,
                              const double height) {
    return new ceres::AutoDiffCostFunction<AprilTagCostFunctor, 8, 7, 7>(
            new AprilTagCostFunctor(topLeft,
                                   topRight,
                                   bottomLeft,
                                   bottomRight,
                                   width,
                                   height)
            );
  }

  template <typename T>
  bool operator()(const T* const aprilTag, const T* const frame, T* residuals) const {
    typedef Eigen::Matrix<T,3,4> CornersT;
    Sophus::SE3<T> poseAT = Eigen::Map< const Sophus::SE3<T> >(aprilTag);
    Sophus::SE3<T> poseFrame = Eigen::Map< const Sophus::SE3<T> >(frame);

    T width = T(_width);
    T height = T(_height);
    
    // corner points of apriltag:  bottom right    top right      bottom left     top left
    CornersT corners3D;
    corners3D << width/T(2),   width/T(2), -width/T(2),  -width/T(2),
                 height/T(2), -height/T(2), height/T(2), -height/T(2),
                 T(0),         T(0),        T(0),         T(0);
    // transform to world frame, then to camera frame
    corners3D = (poseAT.unit_quaternion().matrix()*corners3D).colwise() + poseAT.translation().matrix();
    corners3D = poseFrame.unit_quaternion().inverse().matrix()*(corners3D.colwise() - poseFrame.translation().matrix());

    Eigen::Matrix<T,2,1> br, tr, bl, tl;
    br << corners3D(0,0)/corners3D(2,0), corners3D(1,0)/corners3D(2,0);
    tr << corners3D(0,1)/corners3D(2,1), corners3D(1,1)/corners3D(2,1);
    bl << corners3D(0,2)/corners3D(2,2), corners3D(1,2)/corners3D(2,2);
    tl << corners3D(0,3)/corners3D(2,3), corners3D(1,3)/corners3D(2,3);

    residuals[0] = br[0] - T(_bottomRight[0]);
    residuals[1] = br[1] - T(_bottomRight[1]);
    residuals[2] = tr[0] - T(_topRight[0]);
    residuals[3] = tr[1] - T(_topRight[1]);
    residuals[4] = bl[0] - T(_bottomLeft[0]);
    residuals[5] = bl[1] - T(_bottomLeft[1]);
    residuals[6] = tl[0] - T(_topLeft[0]);
    residuals[7] = tl[1] - T(_topLeft[1]);

    return true;
  }
};

struct AprilTagCostFunctorDepth {
  const Cornersd _measured;
  const double _width;
  const double _height;

  AprilTagCostFunctorDepth(const Cornersd& measured,
                            const double width,
                            const double height):
      _measured(measured)
      , _width(width)
      , _height(height) {}
  
  static ceres::CostFunction* Create(const Cornersd& measured,
                              const double width,
                              const double height) {
    return new ceres::AutoDiffCostFunction<AprilTagCostFunctorDepth, 12, 7, 7>(
            new AprilTagCostFunctorDepth(measured,
                                        width,
                                        height)
            );
  }

  template <typename T>
  bool operator()(const T* const aprilTag, const T* const frame, T* residuals) const {
    typedef Eigen::Matrix<T,3,4> CornersT;
    Sophus::SE3<T> poseAT = Eigen::Map< const Sophus::SE3<T> >(aprilTag);
    Sophus::SE3<T> poseFrame = Eigen::Map< const Sophus::SE3<T> >(frame);

    T width = T(_width);
    T height = T(_height);
    
    // corner points of apriltag:  bottom right    top right      bottom left     top left
    CornersT corners3D;
    corners3D << width/T(2),   width/T(2), -width/T(2),  -width/T(2),
                 height/T(2), -height/T(2), height/T(2), -height/T(2),
                 T(0),         T(0),        T(0),         T(0);
    // transform to world frame, then to camera frame
    
    corners3D = (poseAT.unit_quaternion().matrix()*corners3D).colwise() + poseAT.translation().matrix();
    corners3D = poseFrame.unit_quaternion().inverse().matrix()*(corners3D.colwise() - poseFrame.translation().matrix());

    CornersT measuredT = _measured.cast<T>();
    CornersT diff = measuredT - corners3D;

    residuals[0] =  diff(0,0);
    residuals[1] =  diff(1,0);
    residuals[2] =  diff(2,0);
    residuals[3] =  diff(0,1);
    residuals[4] =  diff(1,1);
    residuals[5] =  diff(2,1);
    residuals[6] =  diff(0,2);
    residuals[7] =  diff(1,2);
    residuals[8] =  diff(2,2);
    residuals[9] =  diff(0,3);
    residuals[10] = diff(1,3);
    residuals[11] = diff(2,3);

    return true;
  }
};

class AprilTagManager {
public:
  AprilTagManager(const double& width, const double& height);
  ~AprilTagManager();
  void addConstraint(Cornersd detections, size_t frameID, size_t tagID, bool use_depth);
  void setCeresOptions(const ceres::Solver::Options& options);
  void solve();
  Sophus::SE3<double> getTagPose(const size_t& id);
  Sophus::SE3<double> getCameraPose(const size_t& id);

  std::map<size_t, AprilFrame*>& getFrames() {return _frames;}
  std::map<size_t, AprilTag*>& getTags() {return _tags;}

private:
  const double _width;
  const double _height;
  ceres::Problem _problem;
  ceres::Solver::Options _options;

  // stored frame and tag poses
  std::map<size_t, AprilFrame*> _frames;
  std::map<size_t, AprilTag*> _tags;

  // mapping frame ID to vector of detected tag IDs
  std::map<size_t, std::vector<size_t>> _graph;

  void addMonoConstraint(Cornersd detections, AprilFrame* frame, AprilTag* tag);
  void addDepthConstraint(Cornersd detections, AprilFrame* frame, AprilTag* tag);
};

#endif