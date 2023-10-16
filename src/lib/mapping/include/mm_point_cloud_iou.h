#ifndef MM_POINT_CLOUD_IOU_H
#define MM_POINT_CLOUD_IOU_H

#include "map_merger.h"
#include <eigen3/Eigen/Core>
#include "nanoflann.hpp"

typedef nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixX3f> NFPC;

class MMPointCloudIOU : public MapMerger {
public:
    MMPointCloudIOU(float max_lin_vel,
                    float max_angl_vel,
                    size_t nsamples,
                    float step_decay,
                    float lin_step_min,
                    float ang_step_min,
                    float near_thr,
                    float max_radius,
                    uint16_t dmin,
                    uint16_t dmax,
                    size_t max_unmerged);
    Eigen::Matrix4f getTransform();
    void updateTracking(DataPacket* dp);
    void updateLost(DataPacket* dp);
    void updateTrackingUnmerged(DataPacket* dp);
    void updateLostUnmerged(DataPacket* dp);
    bool isMergeReady();
    void setDebugFlag(bool flag);

private:
    float getIOU(Eigen::MatrixX4f& test, NFPC& dst);
    // consider excluding poses that require the robot to move through obstacles
    void getSampleTransforms(std::chrono::microseconds frame_to,
                             std::chrono::microseconds frame_from,
                             std::vector<Eigen::Vector3f>& sample);
    // 3 DOF pose optimization, x,z,yaw - consider 6 DOF pose
    Eigen::Matrix4f CompassSearch(Eigen::Vector3f& init,
                                  float init_iou,
                                  Eigen::Vector3f& init_step,
                                  Eigen::MatrixX4f& src,
                                  NFPC& dst);
    void recoverTransform();
    Eigen::Matrix4f SE2toMat(Eigen::Vector3f& se2);
    void showDebugMap(Eigen::MatrixX4f& src,
                      NFPC& dst);
    void cv_xyzToEig(cv::Mat cv_xyz, Eigen::MatrixX4f& result);
    float Halton(size_t i, size_t b);

    cv::Mat _last_point_cloud;
    cv::Mat _point_cloud_to_merge;
    Eigen::MatrixXf _last_pose;
    Eigen::Matrix4f _recovered_pose;
    bool _hasRecovered;
    bool _mapFinished;
    std::chrono::microseconds _t0;
    std::chrono::microseconds _t1;
    const float _mlinv; 
    const float _manglv;
    const size_t _nsamples;
    const float _step_decay;
    const float _lin_step_min;
    const float _ang_step_min;
    const float _near_thr;
    const float _max_radius;
    const uint16_t _dmin;
    const uint16_t _dmax;
    bool _kdebug;

    size_t _unmerged_count;
    const size_t _max_unmerged;

    std::vector<float> _h1;
    std::vector<float> _h2;
    std::vector<float> _h3;
};


#endif