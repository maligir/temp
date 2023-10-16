#ifndef CERES_PF_PAIRWISE_H
#define CERES_PF_PAIRWISE_H

#include "factory.h"

namespace features {
class PFPairwise : public ProblemFactory<PFPairwise> {
protected:
  template <typename CFF, typename F>
  static void BuildInternal(const std::vector<F*>& matches,
                            ceres::Problem& problem);
};
}

#endif