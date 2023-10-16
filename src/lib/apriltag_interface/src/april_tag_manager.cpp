#include "april_tag_manager.h"

AprilTagManager::AprilTagManager(const double& width,
                                 const double& height): _width(width), _height(height) {}

AprilTagManager::~AprilTagManager() {
  for (auto frame : _frames) {
    delete frame.second;
  }

  for (auto tag : _tags) {
    delete tag.second;
  }
}

void AprilTagManager::setCeresOptions(const ceres::Solver::Options& options) {
  _options = options;
}

void AprilTagManager::addConstraint(Cornersd detections,
                                    size_t frameID,
                                    size_t tagID,
                                    bool use_depth) {
  if (_frames.count(frameID) == 0) {
    _frames[frameID] = new AprilFrame(frameID);
    _graph[frameID] = std::vector<size_t>();
  }

  if (_tags.count(tagID) == 0) {
    _tags[tagID] = new AprilTag(tagID);
  }

  _graph[frameID].push_back(tagID);

  if (use_depth) {
    addDepthConstraint(detections, _frames[frameID], _tags[tagID]);
  } else {
    addMonoConstraint(detections, _frames[frameID], _tags[tagID]);
  }
}

void AprilTagManager::addDepthConstraint(Cornersd detections, AprilFrame* frame, AprilTag* tag) {
  _problem.AddResidualBlock(AprilTagCostFunctorDepth::Create(detections, _width, _height), nullptr, tag->getParams(), frame->getParams());
}

void AprilTagManager::addMonoConstraint(Cornersd detections, AprilFrame* frame, AprilTag* tag) {
  Eigen::Vector2d tl = (detections.block<2,1>(0,0))/detections(2,0);
  Eigen::Vector2d tr = (detections.block<2,1>(0,1))/detections(2,1);
  Eigen::Vector2d bl = (detections.block<2,1>(0,2))/detections(2,2);
  Eigen::Vector2d br = (detections.block<2,1>(0,3))/detections(2,3);
  _problem.AddResidualBlock(AprilTagCostFunctor::Create(tl, tr, bl, br, _width, _height), nullptr, tag->getParams(), frame->getParams());
}

void AprilTagManager::solve() {
  ceres::Solver::Summary summary;
  ceres::Solve(_options, &_problem, &summary);
  std::cout << summary.FullReport() << std::endl;
}

Sophus::SE3<double> AprilTagManager::getTagPose(const size_t& id) {
  AprilTag* tag = _tags[id];
  Sophus::SE3<double> map = Eigen::Map<Sophus::SE3<double>>(tag->getParams());
  return map;
}

Sophus::SE3<double> AprilTagManager::getCameraPose(const size_t& id) {
  AprilFrame* frame = _frames[id];
  Sophus::SE3<double> map = Eigen::Map<Sophus::SE3<double>>(frame->getParams());
  return map;
}