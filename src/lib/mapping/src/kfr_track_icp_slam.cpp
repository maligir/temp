#include "kfr_track_icp_slam.h"
#include "kinect_packet.h"
#include "orbslam_utils.h"
#include "ceres_utils.h"
#include "Tracking.h"

#include <k4a/k4a.hpp>
#include <vector>

#include <ceres/types.h>
#include <ceres/rotation.h>
#include <ceres/ceres.h>

#include <ceres/loss_function.h>

KFRTrackICPSLAM::KFRTrackICPSLAM(float dmin,
                                 float dmax,
                                 ORB_SLAM3::System& ORB_SLAM):
    _ORB_SLAM(ORB_SLAM)
    , _kfr(NULL)
    , _patch(5)
    , _dist_th(0.005)
    , _dmin(dmin)
    , _dmax(dmax)
    , _iters(20)
    , _knearest(5) {}

KFRTrackICPSLAM::~KFRTrackICPSLAM() {}

void KFRTrackICPSLAM::setKinectFrameRecipient(KinectFrameRecipient* kfr) {
    _kfr = kfr;
}

void KFRTrackICPSLAM::initializePacket(KinectPacket* kp) {
    if (_kfr) {
        _kfr->initializePacket(kp);
    }
}

void KFRTrackICPSLAM::receiveFrame(KinectPacket* kp) {
    uint8_t orb_state = _ORB_SLAM.GetCurrentState();
    if (orb_state == ORB_SLAM3::Tracking::LOST 
     || orb_state == ORB_SLAM3::Tracking::RECENTLY_LOST) {
        getTransform(kp);
    }

    update(kp);

    if (_kfr) {
        _kfr->receiveFrame(kp);
    }
}

void KFRTrackICPSLAM::update(KinectPacket* kp) {
    k4a::image& img = kp->_images["PointCloudImg"];
    _last_pointcloud = k4a::image::create(img.get_format(),
                                          img.get_width_pixels(),
                                          img.get_height_pixels(),
                                          img.get_stride_bytes());
    std::memcpy(_last_pointcloud.get_buffer(), img.get_buffer(), img.get_size());
    _last_pose = Eigen::MatrixXf(kp->_eigs["Tcw"]);
}

size_t KFRTrackICPSLAM::xy2row(size_t h,
                               size_t w,
                               size_t y,
                               size_t x) {
    return x*w + y;
}

void KFRTrackICPSLAM::row2xy(size_t h,
                             size_t w,
                             size_t row,
                             int& x,
                             int& y) {
    x = row / w;
    y = row % w;
}

void KFRTrackICPSLAM::getCorrespondence(const cv::Mat& src,
                                        const cv::Mat& dst,
                                        size_t h, size_t w,
                                        std::map<size_t, size_t>& corr) {
    size_t nrow = src.rows;
    for (size_t r1 = 0; r1 < nrow; r1++) {
        cv::Mat row1 = src.row(r1);
        float Z1 = row1.at<float>(2);
        if (Z1 < _dmin || Z1 > _dmax) {
            continue;
        }
        float X1 = row1.at<float>(0);
        float Y1 = row1.at<float>(1);
        int x1, y1;
        row2xy(h,w,r1,x1,y1);
        size_t xmin = std::max(0, x1 - _patch);
        size_t xmax = std::min(static_cast<int>(h)-1, x1 + _patch);
        size_t ymin = std::max(0, y1 - _patch);
        size_t ymax = std::min(static_cast<int>(w)-1, y1 + _patch);

        float best_dist = _dist_th*_dist_th + 1;
        size_t best_row = nrow + 1;
        for (size_t x2 = xmin; x2 <= xmax; x2++) {
            for (size_t y2 = ymin; y2 <= ymax; y2++) {
                size_t r2 = xy2row(h,w,y2,x2);
                cv::Mat row2 = dst.row(r2);
                float Z2 = row2.at<float>(2);
                if (Z2 < _dmin || Z2 > _dmax) {
                    continue;
                }
                float X2 = row2.at<float>(0);
                float Y2 = row2.at<float>(1);

                float dX = X2 - X1;
                float dY = Y2 - Y1;
                float dZ = Z2 - Z1;
                float distSq = dX*dX + dY*dY + dZ*dZ;
                if (distSq < best_dist) {
                    best_dist = distSq;
                    best_row = r2;
                }
            }
        }

        if (best_row >= nrow || best_dist > _dist_th*_dist_th) {
            continue;
        }

        corr[r1] = best_row;
    }
}

void KFRTrackICPSLAM::getGeometricFeatures(const NFPC& src,
                                           const NFPC& dst,
                                           std::vector<float>& src_angle,
                                           std::vector<float>& dst_angle,
                                           std::vector<float>& src_curva,
                                           std::vector<float>& dst_curva) {
    size_t src_size = src.index->size(*src.index);
    size_t dst_size = dst.index->size(*dst.index);
    src_angle.reserve(src_size);
    src_curva.reserve(src_size);
    dst_angle.reserve(src_size);
    dst_curva.reserve(src_size);

    for (size_t i = 0; i < src_size; i++) {
        std::vector<float> query_pt(3);
        query_pt[0] = src.kdtree_get_pt(i, 0);
        query_pt[1] = src.kdtree_get_pt(i, 1);
        query_pt[2] = src.kdtree_get_pt(i, 2);

        // do a knn search
        const size_t        num_results = 3;
        std::vector<size_t> ret_indexes(_knearest);
        std::vector<float>  out_dists_sqr(_knearest);
        nanoflann::KNNResultSet<float> resultSet(_knearest);
        resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);
        src.index->findNeighbors(
            resultSet, &query_pt[0], nanoflann::SearchParams(10));
    }

    for (size_t i = 0; i < dst_size; i++) {
        std::vector<float> query_pt(3);
        query_pt[0] = dst.kdtree_get_pt(i, 0);
        query_pt[1] = dst.kdtree_get_pt(i, 1);
        query_pt[2] = dst.kdtree_get_pt(i, 2);


    }
}

void KFRTrackICPSLAM::getTransform(cv::Mat& cv_src,
                                   cv::Mat& cv_dst,
                                   std::map<size_t, size_t>& src_to_dst_corr,
                                   Eigen::Matrix4f& transform) {
    map<size_t, size_t>::iterator it;
    std::vector<Eigen::Vector3d> src;
    std::vector<Eigen::Vector3d> dst;
    for (it = src_to_dst_corr.begin(); it != src_to_dst_corr.end(); it++) {
        cv::Mat cv_src_row = cv_src.row(it->first);
        cv::Mat cv_dst_row = cv_dst.row(it->second);

        if (cv_src_row.type() == CV_16UC1) { 
            Eigen::Vector3d eig_src(cv_src_row.at<uint16_t>(0),
                                    cv_src_row.at<uint16_t>(1),
                                    cv_src_row.at<uint16_t>(2));
            Eigen::Vector3d eig_dst(cv_dst_row.at<uint16_t>(0),
                                    cv_dst_row.at<uint16_t>(1),
                                    cv_dst_row.at<uint16_t>(2));
            
            src.push_back(eig_src / 1000.);
            dst.push_back(eig_dst / 1000.);
        } else {
            Eigen::Vector3d eig_src(cv_src_row.at<float>(0),
                                    cv_src_row.at<float>(1),
                                    cv_src_row.at<float>(2));
            Eigen::Vector3d eig_dst(cv_dst_row.at<float>(0),
                                    cv_dst_row.at<float>(1),
                                    cv_dst_row.at<float>(2));
            
            src.push_back(eig_src);
            dst.push_back(eig_dst);


            // std::cout << "src: " << eig_src.transpose()
            //           << " dst: " << eig_dst.transpose()
            //           << " sq: " << (1000*eig_src - 1000*eig_dst).squaredNorm()
            //           << " src row: " << it->first << " dst row: " << it->second << std::endl;
        }
    }

    Eigen::Matrix4f pose = pointToPoint_SophusSE3(src,dst).matrix().cast<float>();
    kp->_eigs["Tcw"] = _last_pose * pose.inverse();

    /*
    double cam[6] = {0,0,0,0,0,0};
    ceres::Problem problem;
    for (size_t i = 0; i < src.size(); i++) {
        // std::cout << "src: " << src[i].transpose()
        //           << " dst: " << dst[i].transpose()
        //           << " sq: " << (1000*src[i] - 1000*dst[i]).squaredNorm() << std::endl;
        ceres::CostFunction* cost_function = PointToPointError_CeresAngleAxis::Create(dst[i],src[i]);
        problem.AddResidualBlock(cost_function, NULL, cam);
    }
    ceres::Solver::Summary summary;
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.use_explicit_schur_complement=true;
    options.max_num_iterations = 50;
    ceres::Solve(options, &problem, &summary);
    // std::cout << summary.BriefReport() << std::endl;

    Eigen::Matrix4f pose = Eigen::MatrixXf::Identity(4,4);
    pose(0,3) = cam[3];
    pose(1,3) = cam[4];
    pose(2,3) = cam[5];

    Eigen::Quaternionf quat = Eigen::AngleAxisf(cam[0], Eigen::Vector3f::UnitX())
                            * Eigen::AngleAxisf(cam[1], Eigen::Vector3f::UnitY())
                            * Eigen::AngleAxisf(cam[2], Eigen::Vector3f::UnitZ());
    
    Eigen::Matrix3f R = quat.toRotationMatrix();
    pose.block<3,3>(0,0) = R;
    transform = _last_pose * pose.inverse();
}

void KFRTrackICPSLAM::icp(cv::Mat src_xyz, cv::Mat dst_xyz, Eigen::Matrix4f& transform) {
    transform = Eigen::Matrix4f::Identity(4,4);
    size_t h = src_xyz.rows;
    size_t w = src_xyz.cols;
    cv::Mat cv_dst_flat = dst_xyz.reshape(1,h*w);
    cv::Mat cv_src_flat = src_xyz.reshape(1,h*w);

    cv::Mat cv_src_float, cv_dst_float;
    cv_dst_flat.convertTo(cv_src_float, CV_32FC1, 1e-3);
    cv_src_flat.convertTo(cv_dst_float, CV_32FC1, 1e-3);
    
    cv::Mat cv_src_augment;
    cv::Mat augment = cv::Mat::ones(cv_src_float.rows, 1, CV_32FC1);
    cv::hconcat(cv_src_float, augment, cv_src_augment);
    Eigen::MatrixXf eig_src; // NX4
    cv::cv2eigen(cv_src_augment, eig_src);

    for (uint16_t k = 0; k < _knearest; k++) {
        // std::cout << "iteration: " << k+1 << std::endl;
        Eigen::MatrixXf prod = transform * eig_src.transpose();
        Eigen::MatrixXf block = prod.block(0,0,3,prod.cols()).transpose();
        cv::Mat src_transed; // NX3 - float
        cv::eigen2cv(block, src_transed);

        std::map<size_t, size_t> src_to_dst_corr;
        getCorrespondence(src_transed, cv_dst_float, h, w, src_to_dst_corr);
        // std::cout << "icp solver found " << src_to_dst_corr.size() << " corresponding points\n";

        getTransform(src_transed, cv_dst_float, src_to_dst_corr, transform);
        // if (k % 10 == 0) {
        //     std::cout << "found transform:\n" << transform << std::endl;
        // }
    }
    // std::cout << "frame to frame transform:\n" << transform << std::endl;
}

void KFRTrackICPSLAM::getTransform(KinectPacket* kp) {
    k4a::image& k4a_new_pc = kp->_images["PointCloudImg"];
    size_t h = k4a_new_pc.get_height_pixels();
    size_t w = k4a_new_pc.get_width_pixels();
    cv::Mat cv_new_xyz(h,
                       w,
                       CV_16UC3,
                       k4a_new_pc.get_buffer());
    cv::Mat cv_old_xyz(h,
                       w,
                       CV_16UC3,
                       _last_pointcloud.get_buffer());
    
    Eigen::Matrix4f result;
    icp(cv_old_xyz, cv_new_xyz, result);
    kp->_eigs["Tcw"] = _last_pose * result.inverse();
}
