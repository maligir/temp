#include "kfr_depth_to_pointcloud.h"
#include "kinect_packet.h"

using namespace k4a;

KFRDepthToPointcloud::KFRDepthToPointcloud(const k4a::calibration* k4a_calibration):
        _k4a_calibration(k4a_calibration) {
    k4a_transformation_ = k4a_transformation_create(_k4a_calibration);
}
KFRDepthToPointcloud::~KFRDepthToPointcloud() {}

void KFRDepthToPointcloud::initializePacket(KinectPacket* kp) {
    size_t dh = _k4a_calibration->depth_camera_calibration.resolution_height;
    size_t dw = _k4a_calibration->depth_camera_calibration.resolution_width;
    size_t ch = _k4a_calibration->color_camera_calibration.resolution_height;
    size_t cw = _k4a_calibration->color_camera_calibration.resolution_width;
    std::cout << "raw width_pixels: " << dw << std::endl;
    std::cout << "converted width_pixels: " << cw << std::endl;
    kp->_images["PointCloudImg"] =
        k4a::image::create(
            K4A_IMAGE_FORMAT_CUSTOM, dw, dh, dw * (int)sizeof(int16_t) * 3);
    kp->_images["PointCloudImg4K"] =
        k4a::image::create(
            K4A_IMAGE_FORMAT_CUSTOM, cw, ch, cw * (int)sizeof(int16_t) * 3);

    kp->_mats["PointCloudImg"] = cv::Mat(dh, dw, CV_16UC3, kp->_images["PointCloudImg"].get_buffer());
    kp->_mats["PointCloudImg4K"] = cv::Mat(ch, cw, CV_16UC3, kp->_images["PointCloudImg4K"].get_buffer());

    if (_fr) {
        _fr->initializePacket(kp);
    }
}

void KFRDepthToPointcloud::receiveFrame(KinectPacket* kp) {
    k4a::image depth_raw = kp->_capture.get_depth_image();
    k4a_transformation_.depth_image_to_point_cloud(depth_raw, K4A_CALIBRATION_TYPE_DEPTH, &(kp->_images["PointCloudImg"]));
    k4a_transformation_.depth_image_to_point_cloud(kp->_images["Depth4K"], K4A_CALIBRATION_TYPE_COLOR, &(kp->_images["PointCloudImg4K"]));

    if (_fr) {
        _fr->receiveFrame(kp);
    }
}