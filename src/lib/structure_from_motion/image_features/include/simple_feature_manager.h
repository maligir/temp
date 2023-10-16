#ifndef IMAGE_FEATURES_SIMPLE_FEATURE_MANAAGER_H
#define IMAGE_FEATURES_SIMPLE_FEATURE_MANAAGER_H

#include "feature.h"
#include "feature_manager.h"
#include "frame.h"

namespace features {

template <typename F>
class SimpleFM : public FeatureManager<F> {
public:
  void matchAcrossFrames(const size_t src_frameID,
                         const std::vector<size_t>& frameIDs);

  void getMatchingFeaturesInFrame(size_t frameID,
                                  const Features& features,
                                  Features& matches);

  void getFeaturesAndMatchesByFrame(const size_t frameID, MatchedSets& matches);

private:
  void mergeMatchingSets(size_t setAid, size_t setBid);
  void addMatchToSet(size_t setID, FeatureLoc feat_loc);

};

}

#endif