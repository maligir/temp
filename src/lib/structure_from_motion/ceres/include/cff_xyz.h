#ifndef CERES_CFF_XYZ_H
#define CERES_CFF_XYZ_H

#include "factory.h"

namespace features {

class CFFXYZ : public ProblemFactory<XYZFeature, CFFXYZ> {
public:
  template <typename T>
  bool operator()(const T* const cam1, const T* const cam2, T* residuals);

protected:
  CFFXYZ(const XYZFeature* dst, const XYZFeature* src);
  static ceres::CostFunction* CreateInternal(const XYZFeature* dst, const XYZFeature* src);
  static void AddResidualsInternal(const XYZFeature* dst, const XYZFeature* src, ceres::Problem& problem);
};

}

#endif