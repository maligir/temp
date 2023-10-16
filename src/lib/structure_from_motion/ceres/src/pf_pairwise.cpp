#include "pf_pairwise.h"

namespace features {

template <typename CFF, typename F>
void PFPairwise::BuildInternal(const std::vector<F*>& matches, ceres::Problem& problem) {
  for (size_t i = 0; i < matches.size(); i++) {
    for (size_t j = i+1; j < matches.size(); j++) {
      F* dst = matches[i];
      F* src = matches[j];
      CFF::AddResiduals(problem, dst, src);
    }
  }
}

}