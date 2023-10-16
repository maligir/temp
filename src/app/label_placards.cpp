#include "kinect_playback.h"
#include "kinect_frame_recipient.h"
#include "kfr_capture_to_timestamp.h"
#include "kfr_capture_to_bgra.h"
#include "kfr_capture_to_depth.h"
#include "fr_bgra_to_rgb.h"
#include "fr_rgb_4k_to_720p.h"
#include "kfr_depth_to_pointcloud.h"

#include <k4a/k4a.hpp>
#include <eigen3/Eigen/Core>
#include <iostream>
#include <unistd.h>

#include <boost/filesystem.hpp>

#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>
#include <opencv2/opencv.hpp>

#include <fstream>
#include <string>
#include <thread>

void ParsePlacard(std::string& line,
                  Placard& placard) {
    size_t pos = 0;
    std::string token;
    std::string delim = ",";

    // label
    pos = line.find(delim);
    token = line.substr(0, pos);
    line.erase(0, pos + delim.length());
    std::string label = token;

    // confidence
    pos = line.find(delim);
    token = line.substr(0, pos);
    line.erase(0, pos + delim.length());
    float conf = std::stof(token);

    // time stamp
    pos = line.find(delim);
    token = line.substr(0, pos);
    line.erase(0, pos + delim.length());
    size_t ts = std::stoul(token);
    std::chrono::microseconds ms(ts-1);

    // pose
    pos = line.find(delim);
    token = line.substr(0, pos);
    line.erase(0, pos + delim.length());
    Eigen::Vector3f pose;

    {
        std::string sub_token;
        size_t pos2 = token.find(" ");

        // pose x
        while (pos2 == 0) {
            token.erase(0, 1);
            pos2 = token.find(" ");
        }
        sub_token = token.substr(0, pos2);
        token.erase(0, pos2 + 1);
        pose.x() = std::stof(sub_token);
        pos2 = token.find(" ");

        // pose y
        while (pos2 == 0) {
            token.erase(0, 1);
            pos2 = token.find(" ");
        }
        sub_token = token.substr(0, pos2);
        token.erase(0, pos2 + 1);
        pose.y() = std::stof(sub_token);
        pos2 = token.find(" ");

        // pose z
        while (pos2 == 0) {
            token.erase(0, 1);
            pos2 = token.find(" ");
        }
        sub_token = token.substr(0, pos2);
        token.erase(0, pos2 + 1);
        pose.z() = std::stof(sub_token);
    }

    Eigen::Quaternionf quat;
    // x quat
    pos = line.find(delim);
    token = line.substr(0, pos);
    line.erase(0, pos + delim.length());
    quat.x() = std::stof(token);

    // y quat
    pos = line.find(delim);
    token = line.substr(0, pos);
    line.erase(0, pos + delim.length());
    quat.y() = std::stof(token);

    // z quat
    pos = line.find(delim);
    token = line.substr(0, pos);
    line.erase(0, pos + delim.length());
    quat.z() = std::stof(token);

    // w quat
    pos = line.find(delim);
    token = line.substr(0, pos);
    line.erase(0, pos + delim.length());
    quat.w() = std::stof(token);

    placard = Placard(pose, quat, label, conf, ms);
}

void ParseFrame(std::string& line,
                std::chrono::microseconds& ms,
                Eigen::Vector3f& pose,
                Eigen::Quaternionf& quat) {
    size_t pos = 0;
    std::string token;
    std::string delimiter = ",";

    // frame ID
    pos = line.find(delimiter);
    token = line.substr(0, pos);
    line.erase(0, pos + delimiter.length());

    // timestamp
    pos = line.find(delimiter);
    token = line.substr(0, pos);
    line.erase(0, pos + delimiter.length());
    float ts = std::stof(token);
    ms = std::chrono::microseconds(static_cast<size_t>(1e6*ts));

    // x pos
    pos = line.find(delimiter);
    token = line.substr(0, pos);
    line.erase(0, pos + delimiter.length());
    pose.x() = std::stof(token);

    // y pos
    pos = line.find(delimiter);
    token = line.substr(0, pos);
    line.erase(0, pos + delimiter.length());
    pose.y() = std::stof(token);

    // z pos
    pos = line.find(delimiter);
    token = line.substr(0, pos);
    line.erase(0, pos + delimiter.length());
    pose.z() = std::stof(token);

    // x quat
    pos = line.find(delimiter);
    token = line.substr(0, pos);
    line.erase(0, pos + delimiter.length());
    quat.x() = std::stof(token);

    // y quat
    pos = line.find(delimiter);
    token = line.substr(0, pos);
    line.erase(0, pos + delimiter.length());
    quat.y() = std::stof(token);

    // z quat
    pos = line.find(delimiter);
    token = line.substr(0, pos);
    line.erase(0, pos + delimiter.length());
    quat.z() = std::stof(token);

    // w quat
    pos = line.find(delimiter);
    token = line.substr(0, pos);
    line.erase(0, pos + delimiter.length());
    quat.w() = std::stof(token);
}

int main(int argc, char**argv) {
    if (argc < 5) {
        std::cerr << "\nUsage: " << argv[0]
                  << " path_to_k4a_recording"
                  << " path_to_observations"
                  << " path_to_trajectory"
                  << " path_to_data_save_directory\n";
        return 1;
    }

    std::string k4a_rec(argv[1]);
    std::string observations(argv[2]);
    std::string trajectory(argv[3]);
    std::string trial_dir(argv[4]);

    Eigen::Matrix4f axis_flip = Eigen::MatrixXf::Zero(4,4);
    axis_flip(0,0) = 1;
    axis_flip(1,2) = 1;
    axis_flip(2,1) = -1;
    axis_flip(3,3) = 1;

    float fx = 1959.466552734375;
    float fy = 1959.18603515625;
    float cx = 2043.764404296875;
    float cy = 1559.04150390625;
    Eigen::Matrix3f K = Eigen::MatrixXf::Identity(3,3);
    K(0,0) = fx;
    K(1,1) = fy;
    K(0,2) = cx;
    K(1,2) = cy;

    KinectPlayback playback(k4a_rec);

    KFRCaptureToTimeStamp kfrCaptureToTimeStamp;
    KFRCaptureToBGRA kfrCaptureToBGRA;
    
    kfrCaptureToTimeStamp.setFrameRecipient((FrameRecipient*)&kfrCaptureToBGRA);
    playback.setFrameRecipient((FrameRecipient*)&kfrCaptureToTimeStamp);

    playback.start();

    std::string placard_line;
    std::string frame_line;

    std::string record_label = trial_dir + "placard_ground_truth.txt";

    std::ifstream observations_f(observations);
    std::ifstream trajectory_f(trajectory);

    std::chrono::microseconds frame_time;
    Eigen::Vector3f frame_pose;
    Eigen::Quaternionf frame_quat;
    std::getline(trajectory_f, frame_line);
    ParseFrame(frame_line,
               frame_time,
               frame_pose,
               frame_quat);

    while (std::getline(observations_f, placard_line)) {
        Placard placard;
        ParsePlacard(placard_line, placard);
        if (placard.obsv_time_.count() < 500000000) {
            continue;
        }

        while (frame_time < placard.obsv_time_) {
            std::getline(trajectory_f, frame_line);
            ParseFrame(frame_line,
                    frame_time,
                    frame_pose,
                    frame_quat);
        }

        std::cout << "frame time: " << frame_time.count()
                      << ", observation time: " << placard.obsv_time_.count() << std::endl;

        playback.seekFrame(placard.obsv_time_);
        playback.doOnce();

        Eigen::Affine3f kf_pose(frame_quat);
        kf_pose.translation() = frame_pose;
        Eigen::Affine3f rgb_to_world = Eigen::Affine3f(axis_flip)*kf_pose;
        Eigen::Affine3f placard_pose(placard.orientation_);
        placard_pose.translation() = placard.position_;

        Eigen::Affine3f placard_pose_rgb = rgb_to_world.inverse() * placard_pose;
        std::cout << "camera pose in world frame:\n" << kf_pose.matrix() << std::endl;
        std::cout << "placard pose in world frame:\n" << placard_pose.matrix() << std::endl;
        std::cout << "placard pose in camera frame:\n" << placard_pose_rgb.matrix() << std::endl;

        Eigen::Vector3f center = placard_pose_rgb.translation();
        Eigen::Matrix3f R = placard_pose_rgb.rotation();
        Eigen::Matrix<float, 3, 4> corners;
        corners.block<3,1>(0,0) = center - .151 * R.block<3,1>(0,1);
        corners.block<3,1>(0,1) = center + .151 * R.block<3,1>(0,1);
        corners.block<3,1>(0,2) = center - .051 * R.block<3,1>(0,2);
        corners.block<3,1>(0,3) = center + .051 * R.block<3,1>(0,2);

        std::cout << "camera frame corner points:\n" << corners << std::endl;
        corners.array().rowwise() /= corners.row(2).array();
        corners = K * corners;
        std::cout << "projected corner points:\n" << corners << std::endl;
        float minx = std::max(corners.row(0).minCoeff(), (float) 0.);
        float maxx = std::min(corners.row(0).maxCoeff(), (float) 4096);
        float miny = std::max(corners.row(1).minCoeff(), (float) 0.);
        float maxy = std::min(corners.row(1).maxCoeff(), (float) 3072);

        cv::Rect roi(static_cast<int>(minx + 0.5),
                     static_cast<int>(miny + 0.5),
                     static_cast<int>(maxx - minx + 0.5),
                     static_cast<int>(maxy - miny + 0.5));
        std::cout << "roi: (" << roi.x << "," << roi.y << "): "
                  << roi.width << "x" << roi.height << std::endl;
        if (roi.width <= 0 || roi.height <= 0) {
            continue;
        }
        cv::Mat patch = playback.getDataPacket()._mats["BGRA4K"](roi);
        cv::imshow("image patch", patch);
        cv::waitKey(1000);
    }

    return 0;
}