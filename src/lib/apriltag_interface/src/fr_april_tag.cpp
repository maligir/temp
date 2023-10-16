#include "fr_april_tag.h"
#include "data_packet.h"
#include "opencv2/opencv.hpp"
#include <apriltag/apriltag.h>
#include <apriltag/tagStandard41h12.h>
#include <apriltag/tag36h11.h>

using namespace cv;
using namespace std;


FRAprilTag::FRAprilTag(AprilTagManager* manager,
                        const std::string& rgb_cam,
                        const std::string& depth_cam):
         _manager(manager)
         , _frame_count(0)
         , _rgb_cam(rgb_cam)
         , _depth_cam(depth_cam)
         , _use_depth(depth_cam != "") {
    td = apriltag_detector_create();
    tf = tag36h11_create();
    apriltag_detector_add_family(td, tf);
}

void FRAprilTag::initializePacket(DataPacket* dp) {
    if (_fr) {
        _fr->initializePacket(dp);
    }
}

void FRAprilTag::receiveFrame(DataPacket *dp) {
    detectCorners(dp->_mats[_rgb_cam], dp);

    if (_fr) {
        _fr->receiveFrame(dp);
    }
}

void FRAprilTag::detectCorners(cv::Mat img, DataPacket *dp){
    _frame_count++;
    Mat gray;
    cvtColor(img, gray, COLOR_RGB2GRAY);
    image_u8_t im = {.width = gray.cols,
                     .height = gray.rows,
                     .stride = gray.cols,
                     .buf = gray.data};

    zarray_t* tags = apriltag_detector_detect(td, &im);

    Eigen::Matrix3d Kinv  = dp->_eigs[_rgb_cam+"_Kinv"].block<3,3>(0,0).cast<double>();

    for (int i = 0; i < zarray_size(tags); i++){
        apriltag_detection_t *det;
        zarray_get(tags, i, &det);

        Eigen::Matrix<double, 2, 4> detectedCorners;
        for (int i = 0; i < 4; i++){
            detectedCorners(0, i) = det->p[i][0];
            detectedCorners(1, i) = det->p[i][1];
        }
        
        Cornersd corners;
        corners.block<1,4>(2,0) << 1, 1, 1, 1;

        // swap corner order for compatibility with manager class
        corners.block<2,1>(0,0) = detectedCorners.col(0);
        corners.block<2,1>(0,2) = detectedCorners.col(1);
        corners.block<2,1>(0,3) = detectedCorners.col(2);
        corners.block<2,1>(0,1) = detectedCorners.col(3);

        corners = Kinv.block<3,3>(0,0)*corners;

        if (_use_depth) {
            corners.block<3,1>(0,0) *= dp->_mats[_depth_cam].at<int16_t>(detectedCorners(0, 0), detectedCorners(1, 0));
            corners.block<3,1>(0,2) *= dp->_mats[_depth_cam].at<int16_t>(detectedCorners(0, 1), detectedCorners(1, 1));
            corners.block<3,1>(0,3) *= dp->_mats[_depth_cam].at<int16_t>(detectedCorners(0, 2), detectedCorners(1, 2));
            corners.block<3,1>(0,1) *= dp->_mats[_depth_cam].at<int16_t>(detectedCorners(0, 3), detectedCorners(1, 3));
        }

        dp->_aprilTagDetections[det->id] = detectedCorners;
        _manager->addConstraint(corners, _frame_count, det->id, _use_depth);
    }
}
