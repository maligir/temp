#include "kfr_capture_to_depth.h"
#include "kinect_packet.h"

#include <cstring>

using namespace k4a;

KFRCaptureToDepth::KFRCaptureToDepth(const k4a::calibration* k4a_calibration):
        k4a_calibration_(k4a_calibration) {
    k4a_transformation_ = k4a_transformation_create(k4a_calibration_);
}

KFRCaptureToDepth::~KFRCaptureToDepth() {}

void KFRCaptureToDepth::initializePacket(KinectPacket *kp) {
    int w = k4a_calibration_->color_camera_calibration.resolution_width;
    int h = k4a_calibration_->color_camera_calibration.resolution_height;
    kp->_mats["Depth4K"] = cv::Mat(w, h, CV_32F);
    kp->_images["Depth4K"] = k4a::image::create(
        K4A_IMAGE_FORMAT_DEPTH16, w, h, w * (int)sizeof(uint16_t));
    if (_fr) {
        _fr->initializePacket(kp);
    }
}

void KFRCaptureToDepth::receiveFrame(KinectPacket *kp) {
    k4a::image depth_raw = kp->_capture.get_depth_image();

    k4a_transformation_.depth_image_to_color_camera(depth_raw, &kp->_images["Depth4K"]);

    cv::Mat depth_frame_buffer_mat(kp->_images["Depth4K"].get_height_pixels(),
                                   kp->_images["Depth4K"].get_width_pixels(),
                                   CV_16UC1,
                                   kp->_images["Depth4K"].get_buffer());
    depth_frame_buffer_mat.convertTo(kp->_mats["Depth4K"], CV_32FC1, 1.0 / 1000.0f);

    if (_fr) {
        _fr->receiveFrame(kp);
    }
}