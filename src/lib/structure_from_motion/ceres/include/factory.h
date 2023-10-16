#ifndef CERES_FACTORY_H
#define CERES_FACTORY_H

#include "feature.h"

#include <ceres/autodiff_cost_function.h>
#include <ceres/types.h>
#include <ceres/ceres.h>
#include <sophus/se3.hpp>
#include <vector>

namespace features {

template <class F, class CFF>
class CostFunctorFactory {
static_assert(std::is_base_of<Feature, F>::value,
              "Template class F is not of type Feature");
static_assert(std::is_same<CFF, decltype(*this)>::value,
              "Derived class does not match template argument CFF.");
static_assert(std::is_base_of<CostFunctorFactory, CFF>::value,
              "Template class CFF is not of type CostFunctorFactory");
public:
  static ceres::CostFunction* Create(const F* dst, const F* src) {
    return CFF::CreateInternal(dst, src);
  }

  static void AddResiduals(const F* dst, const F* src, ceres::Problem& problem) {
    CFF::AddResidualsInternal(dst, src, problem);
  }
  
protected:
  CostFunctorFactory(const F* dst, const F* src) : _dst(dst), _src(src) {}
  const F* _dst;
  const F* _src;
};

template <class PF>
class ProblemFactory {
static_assert(std::is_same<PF, decltype(*this)>::value,
              "Derived class does not match the template argument.");
static_assert(std::is_base_of<ProblemFactory, FP>::value,
              "Template class PF is not of type ProblemFactory");
public:
  template <typename CFF, typename F>
  static void BuildProblem(const std::vector<F*>& matches,
                           ceres::Problem& problem) {
    static_assert(std::is_base_of<CostFunctorFactory, CFF>::value,
                  "Template class CFF is not of type CostFunctorFactory");
    static_assert(std::is_base_of<Feature, F>::value,
                  "Template class F is not of type Feature");
    PF::BuildInternal<CFF, F>(matches, problem);
  }
};

}
#endif