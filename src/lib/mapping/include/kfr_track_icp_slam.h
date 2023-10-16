#ifndef KFR_TRACK_ICP_SLAM_H
#define KFR_TRACK_ICP_SLAM_H

#include "kinect_frame_recipient.h"
#include "System.h"

#include <k4a/k4a.hpp>
#include <octomap/Pointcloud.h>
#include <eigen3/Eigen/Core>
#include "nanoflann.hpp"

typedef nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXf> NFPC;

class KFRTrackICPSLAM : public KinectFrameRecipient {
public:
    KFRTrackICPSLAM(float dmin,
                    float dmax,
                    ORB_SLAM3::System& ORB_SLAM);
    ~KFRTrackICPSLAM();
    void setKinectFrameRecipient(KinectFrameRecipient* kfr);
    void initializePacket(KinectPacket *kp);

    // checks if ORBSLAM lost track of frame
    // if not, updates local data from packet
    // if so, runs ICP algorithm between _last_pointcloud and packet's pointcloud
    // updates ORBSLAM's current frame with found transform
    // once complete, updates local data from packet
    void receiveFrame(KinectPacket *kp);

    void icp(cv::Mat src_xyz, cv::Mat dst_xyz, Eigen::Matrix4f& transform);
    void getTransform(KinectPacket* kp);
    void getTransform(cv::Mat& cv_src,
                      cv::Mat& cv_dst,
                      std::map<size_t, size_t>& src_to_dst_corr,
                      Eigen::Matrix4f& transform);
    void update(KinectPacket* kp);

protected:
    size_t xy2row(size_t h, size_t w, size_t y, size_t x);
    void row2xy(size_t h, size_t w, size_t row, int& x, int& y);
    void getCorrespondence(const cv::Mat& src,
                           const cv::Mat& dst,
                           size_t h,
                           size_t w,
                           std::map<size_t, size_t>& corr);
    void getGeometricFeatures(const NFPC& src,
                              const NFPC& dst,
                              std::vector<float>& src_angle,
                              std::vector<float>& dst_angle,
                              std::vector<float>& src_curva,
                              std::vector<float>& dst_curva);

    KinectFrameRecipient* _kfr;
    ORB_SLAM3::System& _ORB_SLAM;
    k4a::image _last_pointcloud;
    Eigen::Matrix4f _last_pose;
    uint16_t _patch;
    float _dist_th;
    float _dmin;
    float _dmax;
    uint16_t _knearest;
    uint16_t _iters;
};

#endif
