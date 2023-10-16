#include "System.h"
#include "Map.h"
#include "KeyFrame.h"
#include "kinect_playback.h"
#include "kinect_frame_recipient.h"
#include "kfr_capture_to_timestamp.h"
#include "kfr_capture_to_bgra.h"
#include "kfr_capture_to_depth.h"
#include "kfr_bgra_to_rgb.h"
#include "kfr_track_orb_slam.h"
#include "kfr_depth_map_filter.h"
#include "kfr_depth_to_pointcloud.h"
#include "kfr_track_icp_slam.h"
#include "orbslam_utils.h"
#include "octmap_utils.h"
#include "kfr_rgb_4k_to_720p.h"

#include <k4a/k4a.hpp>
#include <eigen3/Eigen/Core>
#include <unistd.h>

int main(int argc, char**argv) {

    if (argc < 4) {
        std::cerr << "\nUsage: ./test_icp path_to_orb_vocabulary path_to_orb_settings path_to_k4a_recording\n";
        return 1;
    }
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::RGBD,true);
    const float depth_min = 0.25;
    const float depth_max = 2.88;

    KinectPlayback playback(argv[3]);
    playback.seekFrame(std::chrono::microseconds{5947911});
    k4a::calibration k4a_calibration = playback.GetCalibration();

    KFRCaptureToTimeStamp kfrCaptureToTimeStamp;
    KFRCaptureToBGRA kfrCaptureToBGRA;
    KFRBGRAToRGB KFRBGRAToRGB;
    KFRCaptureToDepth kfrCaptureToDepth(k4a_calibration);
    KFRDepthMapFilter kfrDepthMapFilter(depth_min, depth_max);
    KFRDepthToPointcloud kfrDepthToPointcloud(k4a_calibration);
    KFRRGB4KTo720p kfrRGB4KTo720p;
    KFRTrackORBSLAM kfrTrackORBSLAM(SLAM, false);
    KFRTrackICPSLAM kfrTrackICPSLAM(depth_min,
                                    depth_max,
                                    SLAM);

    // kfrRGB4KTo720p.setKinectFrameRecipient((KinectFrameRecipient*)&kfrTrackORBSLAM);
    // kfrDepthToPointcloud.setKinectFrameRecipient((KinectFrameRecipient*)&kfrRGB4KTo720p);
    kfrDepthMapFilter.setKinectFrameRecipient((KinectFrameRecipient*)&kfrDepthToPointcloud);
    kfrCaptureToDepth.setKinectFrameRecipient((KinectFrameRecipient*)&kfrDepthMapFilter);
    KFRBGRAToRGB.setKinectFrameRecipient((KinectFrameRecipient*)&kfrCaptureToDepth);
    kfrCaptureToBGRA.setKinectFrameRecipient((KinectFrameRecipient*)&KFRBGRAToRGB);
    kfrCaptureToTimeStamp.setKinectFrameRecipient((KinectFrameRecipient*)&kfrCaptureToBGRA);
    playback.setKinectFrameRecipient((KinectFrameRecipient*)&kfrCaptureToTimeStamp);

    playback.start();

    playback.doOnce();
    playback._pkt._eigs["Tcw"] = Eigen::MatrixXf::Identity(4,4);
    kfrTrackICPSLAM.update(&playback._pkt);

    // cv::Mat points;
    // GetPointCloudInMapFrame(playback._pkt._eigs["Tcw"], &(playback._pkt), 250, 2880, points);
    // Eigen::Matrix4f eig_test = Eigen::MatrixXf::Identity(4,4);
    // eig_test(2,3) = 1;

    // cv::Mat augment = cv::Mat::ones(points.rows, 1, CV_32FC1);
    // cv::Mat augmented;
    // cv::hconcat(points, augment, augmented);
    // Eigen::MatrixXf eig_cloud;
    // cv::cv2eigen(augmented, eig_cloud);
    // Eigen::MatrixXf prod = eig_test * eig_cloud.transpose();
    // Eigen::MatrixXf block = prod.block(0,0,3,prod.cols()).transpose();
    // cv::Mat pts_test;
    // cv::eigen2cv(block, pts_test);

    // std::map<size_t, size_t> src_to_dst_corr;
    // for (size_t row = 0; row < pts_test.rows; row++) {
    //     src_to_dst_corr[row] = row;
    // }

    // std::cout << "original point:\n" << points.row(0) << std::endl;
    // std::cout << "transformed point:\n" << pts_test.row(0) << std::endl;

    // Eigen::Matrix4f result;
    // kfrTrackICPSLAM.getTransform(points, pts_test, src_to_dst_corr, result);
    // std::cout << "true transform:\n" << eig_test << std::endl;
    // std::cout << "found transform:\n" << result.inverse() << std::endl;

    size_t count = 0;
    while (!playback.isFinished() && count  < 100) {
        count++;
        playback.doOnce();
        // std::cout << "ORB-SLAM Pose:\n" 
        //           << playback._pkt._eigs["Tcw"] << std::endl;
        if (count == 1) {
            playback._pkt._eigs["Tcw"] = Eigen::MatrixXf::Identity(4,4);
        }
        if (count > 1) {
            kfrTrackICPSLAM.getTransform(&playback._pkt);
            std::cout << "ICP-SLAM Pose:\n" 
                      << playback._pkt._eigs["Tcw"] << std::endl;
        }
        kfrTrackICPSLAM.update(&playback._pkt);
    }
    SLAM.Shutdown();

    std::cout << "finished\n";
}