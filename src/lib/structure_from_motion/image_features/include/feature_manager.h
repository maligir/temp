#ifndef IMAGE_FEATURES_FEATURE_MANAGER_H
#define IMAGE_FEATURES_FEATURE_MANAGER_H

#include <vector>
#include <pair>

#include "bimap.h"
#include "feature.h"
#include "feature_extractor.h"

namespace features {

// Class for extracting features from an image
template <typename F>
class FeatureExtractor {
public:
  FeatureExtractor() {
    static_assert(std::is_base_of<Feature, F>::value,
                  "Template type F must inherit from Feature.");
  }
  virtual std::vector<const F*>* extractFeatures(const Frame* frame) const = 0;
};

template <typename F>
class FeatureMatcher {
public:
  FeatureMatcher() {
    static_assert(std::is_base_of<Feature, F>::value,
                  "Template type F must inherit from Feature.");
  }

  // returns map from srcIdx to poolIdx
  virtual void findMatches(const std::vector<const F*>* src,
                           const std::vector<const F*>* pool,
                           std::map<size_t, size_t>& matches) const = 0;
};

class FeatureManagerBase {
public:
  virtual void handleFrame(Frame* frame) = 0;
};

template <typename F>
class FeatureManager : public FeatureManagerBase {

typedef std::vector<const F*> Features;
typedef std::vector<Features> MatchedSets;
typedef std::pair<size_t, size_t> FeatureLoc;

public:
  FeatureManager(const FeatureExtractor<F>* fe, const FeatureMatcher<F>* fm): _fe(fe), _fm(fm) {
    static_assert(std::is_base_of<Feature, F>::value,
                  "Template type F must inherit from Feature.");
  }

  ~FeatureManager() {
    // Deallocate memory for _fmap and its contents
    for (auto& pair : _fmap) {
        Features* features = pair.second;
        for (const F* feature : *features) {
            delete feature;
        }
        delete features;
    }
  }

  void handleFrame(Frame* frame) {
    const size_t id = frame->getID();
    if (_frames.count(id) > 0) {
      return;
    }
    Features* features = _fe->extractFeatures(frame);
    _fmap[id] = features;
    _frames[id] = frame;
  }

  // matches features from source frame across one or more frames
  virtual void matchAcrossFrames(const size_t src_frameID,
                                 const std::vector<size_t>& frameIDs) = 0;

  // given a feature, return all known matches to that feature
  // return value gives the index of the given feature in the match set
  size_t getMatchingFeatures(const Feature* feature, Features& matches) {
    FeatureLoc floc = _feature2loc[feature];
    FeatureLoc mloc = _flocs.getRight(floc);
    matches = _matches[mloc.first];
    return mloc.second;
  }

  // given a list of global features, return best matching feature in given frame
  // corresponding vector index contains nullptr if no match found
  virtual void getMatchingFeaturesInFrame(
    size_t frameID,
    const Features& features,
    Features& matches) = 0;

  // given a frame, return all features along with associated matches
  // features grouped by matching set
  void getFeaturesAndMatchesByFrame(const size_t frameID, MatchedSets& feature_matches) {
    Features* features = _fmap[frameID];
    for (size_t i = 0; i < features->size(); i++) {
      Features matches;
      getMatchingFeatures(features[i], matches);
      feature_matches.push_back(matches);
    }
  }

protected:
  const FeatureExtractor<F>* _fe;
  const FeatureMatcher<F>* _fm;

  // map from FrameID to feature vector
  std::map<size_t, Features* > _fmap;
  // map from FrameID to frame
  std::map<size_t, Frame*> _frames;
  // features grouped by matching sets
  MatchedSets _matches;
  // bimap between (FrameID,VectorIndex) and (MatchSet,VectorIndex)
  bimap<FeatureLoc, FeatureLoc> _flocs;
  // map from feature pointer to (FrameID,VectorIndex)
  std::map<Feature*, FeatureLoc> _feature2loc;
};
}
#endif