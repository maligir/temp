#ifndef ORBSLAM_UTILS_H
#define ORBSLAM_UTILS_H

#include "System.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "Converter.h"
#include "data_packet.h"
#include <k4a/k4a.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <iostream>
#include <fstream>

template<class T>
static inline void FilterByRow(const cv::Mat raw,
                               cv::Mat& filtered,
                               T dmin,
                               T dmax) {
    int nrows = raw.rows;
    std::vector<int> rows;
    for (int i = 0; i < nrows; i++) {
        cv::Mat row = raw.row(i);
        T d = row.at<T>(2);
        if (d > dmin && d < dmax) {
            rows.push_back(i);
        }
    }

    filtered.create(rows.size(), raw.cols, raw.type());
    for (int i = 0; i < rows.size(); i++) {
        filtered.row(i).at<T>(0) = raw.row(rows[i]).at<T>(0);
        filtered.row(i).at<T>(1) = raw.row(rows[i]).at<T>(1);
        filtered.row(i).at<T>(2) = raw.row(rows[i]).at<T>(2);
    }
}

static inline void GetPointCloudInMapFrame(const Eigen::Matrix4f& Twc,
                                           DataPacket* packet,
                                           uint16_t dmin,
                                           uint16_t dmax,
                                           cv::Mat& points) {
    cv::Mat cv_xyz = packet->_mats["PointCloudImg"];
    size_t h = cv_xyz.rows;
    size_t w = cv_xyz.cols;

    // convert XYZ image to NX3 point cloud
    cv::Mat cv_flat_cloud = cv_xyz.reshape(1,h*w);
    // depth thresholding
    cv::Mat cv_filtered_cloud;
    FilterByRow(cv_flat_cloud, cv_filtered_cloud, dmin, dmax);
    // scale from mm to m
    cv::Mat cv_float_cloud;
    cv_filtered_cloud.convertTo(cv_float_cloud, CV_32FC1, 1e-3);
    // augment matrix for translation term
    cv::Mat augment = cv::Mat::ones(cv_filtered_cloud.rows, 1, CV_32FC1);
    cv::hconcat(cv_float_cloud, augment, cv_float_cloud);

    // transform from camera frame to map frame
    Eigen::MatrixXf eig_cloud;
    cv::cv2eigen(cv_float_cloud, eig_cloud);
    Eigen::MatrixXf prod = Twc * eig_cloud.transpose();
    Eigen::MatrixXf block = prod.block(0,0,3,prod.cols()).transpose();
    cv::eigen2cv(block, points);
}

static inline void SaveMapToFile(const std::string& fname,
                                 const std::vector<ORB_SLAM3::MapPoint*>& map) {
    std::ofstream fmap;
    fmap.open(fname);

    for (ORB_SLAM3::MapPoint* mp : map) {
        if (mp->isBad()) {
            continue;
        }
        Eigen::Vector3f pose = mp->GetWorldPos();
        Eigen::Vector3f normal = mp->GetNormal();

        fmap << setprecision(9)
             << pose(0) << ","
             << pose(1) << ","
             << pose(2) << ","
             << normal(0) << ","
             << normal(1) << ","
             << normal(2) << "\n";
    }
    fmap.close();
}

static inline void SaveKeyframeTrajectoryToFile(const std::string& fname,
                                                const std::vector<ORB_SLAM3::KeyFrame*>& traject) {
    std::ofstream ftraj;
    ftraj.open(fname);

    for (ORB_SLAM3::KeyFrame* kf : traject) {
        if (kf->isBad()) {
            continue;
        }
        Sophus::SE3f Twc = kf->GetPoseInverse();
        Eigen::Quaternionf q = Twc.unit_quaternion();
        Eigen::Vector3f t = Twc.translation();

        ftraj << kf->mnId << ","
              << setprecision(9)
              << kf->mTimeStamp << ","
              << t(0) << ","
              << t(1) << ","
              << t(2) << ","
              << q.x() << ","
              << q.y() << ","
              << q.z() << ","
              << q.w() << "\n";
    }
    ftraj.close();
}

static inline void SavePointCloudToFile(const std::string& fname, cv::Mat& points) {
    std::cout << "point cloud shape: " << points.rows << "x" << points.cols << std::endl;
    std::ofstream fpts;
    fpts.open(fname, std::ofstream::out | std::ofstream::app);
    fpts << cv::format(points, cv::Formatter::FMT_CSV) << std::endl;
    fpts.close();
}

#endif