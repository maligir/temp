#include "fr_apriltag_pose.h"
#include "opencv2/opencv.hpp"
#include "data_packet.h"
#include <Eigen/Dense>

#include <iostream>

using namespace std;


FRAprilTagPose::FRAprilTagPose() {
    _intrinsics.setAlpha(915.147);
    _intrinsics.setBeta(914.843);
    _intrinsics.setGamma(0);
    _intrinsics.setU0(956.464);
    _intrinsics.setV0(554.229);

    _distortionModel.update(0, 0, 0, .000711092, -.000199659);
}

void FRAprilTagPose::initializePacket(DataPacket* dp) {
    if (_fr) {
        _fr->initializePacket(dp);
    }
}

void FRAprilTagPose::receiveFrame(DataPacket* dp) {

    cv::Mat rgb;
    cv::resize(dp->_mats["BGRA4K"], rgb, cv::Size(1920, 1080));


    dp->_mats["pure"] = cv::Mat(rgb.rows, rgb.cols, CV_8UC3);

    int radius = 5;
    cv::Scalar color(0,255,255);
    int thickness = 5;

    for (auto item : dp->_aprilTagDetections) {

        Eigen::Matrix<double, 2, 4> aprilTagCorners = addARowOfConst(item.second, 1);

        Eigen::MatrixXd model(3,4);

        //Upper Left Coordinate
        model(0,0) = -1;
        model(1,0) = -1;
        model(2,0) = 1;
        //Lower Left Coordinate
        model(0,1) = -1;
        model(1,1) = 1;
        model(2,1) = 1;
        //Lower Right Coordinate
        model(0,2) = 1;
        model(1,2) = 1;
        model(2,2) = 1;
        //Uppter Left Coordinate
        model(0,3) = 1;
        model(1,3) = -1;
        model(2,3) = 1;

        Eigen::MatrixXd coordFrameAxes(4,4);

        coordFrameAxes(0,0) = 0;
        coordFrameAxes(1,0) = 0;
        coordFrameAxes(2,0) = 0;
        coordFrameAxes(3,0) = 1;

        coordFrameAxes(0,1) = 3;
        coordFrameAxes(1,1) = 0;
        coordFrameAxes(2,1) = 0;
        coordFrameAxes(3,1) = 1;

        coordFrameAxes(0,2) = 0;
        coordFrameAxes(1,2) = 3;
        coordFrameAxes(2,2) = 0;
        coordFrameAxes(3,2) = 1;

        coordFrameAxes(0,3) = 0;
        coordFrameAxes(1,3) = 0;
        coordFrameAxes(2,3) = -3;
        coordFrameAxes(3,3) = 1;

        if (aprilTagCorners.cols() > 0) {
            vector<Eigen::MatrixXd> homographies;
            homographies.push_back(computeDLTHomography(model, aprilTagCorners));
            RigidTrans aprilTagRT = extractRTs(_intrinsics, homographies)[0];
            _aprilTagRTMats[item.first] = aprilTagRT.getTransMat();

            Eigen::MatrixXd transformedAxes = projectPts(_intrinsics, _distortionModel, aprilTagRT, coordFrameAxes);

            cv::Point center(transformedAxes.coeff(0,0), transformedAxes.coeff(1,0));
            cv::circle(rgb, center, radius, color, thickness);

            for (int i = 1; i < 4; i++) {
                cv::Point currAxisPoint(transformedAxes.coeff(0,i), transformedAxes.coeff(1,i));
                cv::line(rgb, currAxisPoint, center, color, thickness, cv::LINE_AA);
            }

            Eigen::MatrixXd corners(4,4);

            corners(0,0) = -1;
            corners(1,0) = -1;
            corners(2,0) = 0;
            corners(3,0) = 1;

            corners(0,1) = -1;
            corners(1,1) = 1;
            corners(2,1) = 0;
            corners(3,1) = 1;

            corners(0,2) = 1;
            corners(1,2) = 1;
            corners(2,2) = 0;
            corners(3,2) = 1;

            corners(0,3) = 1;
            corners(1,3) = -1;
            corners(2,3) = 0;
            corners(3,3) = 1;



            Eigen::MatrixXd cornerProj = projectPts(_intrinsics, _distortionModel, aprilTagRT, corners);
            for(int i = 0; i < 4; i++) {
                cv::Point2d curCornerPoint(cornerProj.coeff(0,i), cornerProj.coeff(1,i));
                cv::circle(rgb, curCornerPoint, radius, color, thickness);
            }
        }
    }

    rgb.copyTo(dp->_mats["pure"]);

    if (_fr) {
        _fr->receiveFrame(dp);
    }
}

void FRAprilTagPose::saveTheNextRT(string filename) {
    ofstream file;
    file.open(filename);
    for(auto item : _aprilTagRTMats)
    cout << "id: " << item.first << " Mat: " << item.second << endl;
    file.close();
}