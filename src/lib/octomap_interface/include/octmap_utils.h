#ifndef OCTOMAP_UTILS_H
#define OCTOMAP_UTILS_H

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/Pointcloud.h>

#include <opencv2/opencv.hpp>

#include "occupation_map_2d.h"

static inline void setMapSize(const octomap::OcTree& octo,
                              OccupationMap2D& map) {
    double minx, miny, minz;
    double sizex, sizey, sizez;
    octo.getMetricMin(minx, miny, minz);
    octo.getMetricSize(sizex, sizey, sizez);

    std::pair<float, float> offsets;
    offsets.first = minx - 0.05*sizex;
    offsets.second = miny - 0.05*sizey;
    float res = 1.05*std::max(sizex / map._width, sizey / map._length);

    std::cout << "offsets: " << offsets.first << ", " << offsets.second << std::endl;
    std::cout << "resolution: " << res << std::endl;
    map.SetOffset(offsets.first, offsets.second);
    map.SetResolution(res);
}

static inline void CVMat2OcTreePC(const cv::Mat& mat,
                                  const float cam_height,
                                  octomap::Pointcloud& octo) {
    const size_t rows = mat.rows;
    for (size_t i = 0; i < rows; i++) {
        cv::Mat row = mat.row(i);
        // flip axes so that z-axis is aligned with gravity
        octo.push_back(row.at<float>(0),
                       row.at<float>(2),
                       -row.at<float>(1) - cam_height);
    }
}

static inline bool isSpeckleNode(const octomap::OcTreeKey& nKey,
                                 const octomap::OcTree& octree) {
  octomap::OcTreeKey key;
  bool neighborFound = false;
  for (key[2] = nKey[2] - 1; !neighborFound && key[2] <= nKey[2] + 1; ++key[2]){
    for (key[1] = nKey[1] - 1; !neighborFound && key[1] <= nKey[1] + 1; ++key[1]){
      for (key[0] = nKey[0] - 1; !neighborFound && key[0] <= nKey[0] + 1; ++key[0]){
        if (key != nKey){
          octomap::OcTreeNode* node = octree.search(key);
          if (node && octree.isNodeOccupied(node)){
            // we have a neighbor => break!
            neighborFound = true;
          }
        }
      }
    }
  }

  return !neighborFound;
}


static inline void update2DMap(const octomap::OcTree::iterator& it,
                               const octomap::OcTree& tree,
                               OccupationMap2D& map,
                               unsigned int tree_depth,
                               bool occupied){
    if (it.getDepth() == tree_depth) {
        float x = it.getX();
        float y = it.getY();
        if (!map.isInBounds(x,y)) {
            return;
        }
        size_t idx = map.GetIdx(x, y);
        if (occupied) {
            map.SetValue(idx, OCCUPIED);
        } else if (map.GetValue(idx) == UNKNOWN) {
            map.SetValue(idx, FREE);
        }
    } else {
        int intSize = 1 << (tree_depth - it.getDepth());
        octomap::OcTreeKey minKey=it.getIndexKey();
        for(int dx=0; dx < intSize; dx++) {
            int i = minKey[0] + dx;
            for(int dy=0; dy < intSize; dy++) {
                int j = minKey[1] + dy;
                octomap::OcTreeKey k(i, j, minKey[2]);
                octomap::point3d pt = tree.keyToCoord(k);
                if (!map.isInBounds(pt.x(),pt.y())) {
                    continue;
                }
                size_t idx = map.GetIdx(pt.x(), pt.y());
                if (occupied) {
                    map.SetValue(idx, OCCUPIED);
                } else if (map.GetValue(idx) == UNKNOWN) {
                    map.SetValue(idx, FREE);
                }
            }
        }
    }
}

static inline void OcTree2OccupationMap(const octomap::OcTree& octree,
                                        OccupationMap2D& map,
                                        const float max_floor_height = -0.9,
                                        const float min_ceiling_height = 0.1,
                                        bool filter_speckles = false) {
    setMapSize(octree, map);

    unsigned int tree_depth = octree.getTreeDepth();
    for (octomap::OcTree::iterator it = octree.begin(),
         end = octree.end(); it != end; ++it) {
        unsigned int depth = it.getDepth();
        bool occupied = octree.isNodeOccupied(*it);
        if (!occupied) {
            update2DMap(it, octree, map, tree_depth, occupied);
            continue;
        }
        double z = it.getZ();
        double half_size = it.getSize() / 2.0;
        if (z + half_size > max_floor_height &&
            z - half_size < min_ceiling_height) {

            if (filter_speckles && occupied && (depth == tree_depth + 1)) {
                if (isSpeckleNode(it.getKey(), octree))
                    continue;
            }
            update2DMap(it, octree, map, tree_depth, occupied);
        } 
    }
}

#endif