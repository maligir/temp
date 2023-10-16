#include "simple_feature_manager.h"

namespace features {

void SimpleFM::mergeMatchingSets(size_t setAid, size_t setBid) {
  Features& setA = _matches[setAid];
  Features& setB = _matches[setBid];

  size_t Asize = setA.size();
  for (size_t i = 0; i < setB.size(); i++) {
    FeatureLoc old_mat_loc(setBid, i);
    FeatureLoc new_mat_loc(setAid, Asize + i);
    FeatureLoc old_feat_loc = _flocs.getLeft(old_mat_loc);
    _flocs.remove(old_feat_loc, old_mat_loc);
    _flocs.add(old_feat_loc, new_mat_loc);
  }
  setA.insert(setA.end(), setB.begin(), setB.end());
  setB.clear();
}

void SimpleFM::addMatchToSet(size_t setID, FeatureLoc feat_loc) {
  Features& set = _matches[setID];
  FeatureLoc mat_loc(setID, set.size());
  _flocs.add(feat_loc, mat_loc);
  const Features* features = _fmap[feat_loc.first];
  Feature* feature = features->at(feat_loc.second);
  set.push_back(feature);
}

void SimpleFM::matchAcrossFrames(
    const size_t src_frameID,
    const std::vector<size_t>& frameIDs) {
  Frame* src = _frames[src_frameID];
  const Features* src_features = _fmap[src_frameID];

  for (size_t j = 0; j < frameIDs.size(); j++) {
    const Features* dst_features = _fmap[frameIDs[j]];
    std::map<size_t, size_t> matches_map;
    _fm->findMatches(src_features, dst_features, matches_map);
    for (size_t i = 0; i < src_features->size(); i++) {
      if (matches_map.count(i) <= 0) {
        continue;
      }
      FeatureLoc src_feat_loc(src_frameID, i);
      FeatureLoc dst_feat_loc(frameIDs[j], matches_map[i]);
      const Feature* src_feat = src_features->at(i);

      bool found_src = _flocs.containsLeft(src_feat_loc);
      bool found_dst = _flocs.containsLeft(dst_feat_loc);

      if (found_src && found_dst) {
        FeatureLoc src_mat_loc = _flocs.getRight(src_feat_loc);
        FeatureLoc dst_mat_loc = _flocs.getRight(dst_feat_loc);

        if (src_mat_loc.first != dst_mat_loc.first) {
          mergeMatchingSets(src_mat_loc.first, dst_mat_loc.first);
        }
      } else if (found_src && !found_dst) {
        FeatureLoc src_mat_loc = _flocs.getRight(src_feat_loc);
        addMatchToSet(src_mat_loc.first, dst_feat_loc);
      } else if (!found_src && found_dst) {
        FeatureLoc dst_mat_loc = _flocs.getRight(dst_feat_loc);
        addMatchToSet(dst_mat_loc.first, src_feat_loc);
      } else if (!found_src && !found_dst) {
        const Feature* dst_feat = dst_features->at(dst_feat_loc.second);
        FeatureLoc src_mat_loc(_matches.size(), 0);
        FeatureLoc dst_mat_loc(_matches.size(), 1);
        _flocs.add(src_feat_loc, src_mat_loc);
        _flocs.add(dst_feat_loc, dst_mat_loc);
        _matches.push_back(Features(src_feat, dst_feat));
      }
    }
  }
}

void SimpleFM::getMatchingFeaturesInFrame(
    size_t frameID,
    const Features& features,
    Features& matches) {
  const Features* pool = _fmap[frameID];
  std::map<size_t, size_t> match_map;
  _fm->findMatches(&features, pool, match_map);
  for (size_t i = 0; i < features.size(); i++) {
    if (match_map.count(i) > 0) {
      matches.push_back(pool->at(match_map[i]));
    } else {
      matches.push_back(nullptr);
    }
  }
}

}