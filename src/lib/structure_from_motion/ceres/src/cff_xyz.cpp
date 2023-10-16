#include "pf_xyz.h"

namespace features {

CFFXYZ::CFFXYZ(const XYZFeature* dst, const XYZFeature* src) : _dst(dst), _src(src) {}

ceres::CostFunction* CFFXYZ::CreateInternal(const XYZFeature* dst, const XYZFeature* src) {
  return (new ceres::AutoDiffCostFunction<CFFXYZ, 3, 7, 7>(new CFFXYZ(dst, src)));
}

template<typename T> 
bool CFFXYZ::operator()(const T *cam1, const T *cam2, T *residuals) {
  Eigen::Vector3d src_pnt = _src->getPoint();
  Eigen::Vector3d dst_pnt = _dst->getPoint();
  Eigen::Matrix<T,3,1> p1; p1 << T(src_pnt[0]), T(src_pnt[1]), T(src_pnt[2]);
  Eigen::Matrix<T,3,1> p2; p2 << T(dst_pnt[0]), T(dst_pnt[1]), T(dst_pnt[2]);
  Sophus::SE3<T> q1 = Eigen::Map< const Sophus::SE3<T> >(cam1);
  Sophus::SE3<T> q2 = Eigen::Map< const Sophus::SE3<T> >(cam2);

  p1 = q1.unit_quaternion() * p1 + q1.translation();
  p2 = q2.unit_quaternion() * p2 + q2.translation();

  residuals[0] = p1[0] - p2[0];
  residuals[1] = p1[1] - p2[1];
  residuals[2] = p1[2] - p2[2];
}

static void CFFXYZ::AddResidualsInternal(const XYZFeature* dst, const XYZFeature* src, ceres::Problem& problem) {
  ceres::CostFunction *cost_function = CFFXYZ::CreateInternal(dst, src);
  problem.AddResidualBlock(cost_function,
                          nullptr,
                          dst->getFrame()->getParams(),
                          src->getFrame()->getParams());
}

}