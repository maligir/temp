#include "kfr_depth_to_pointcloud_gpu.h"
#include "kinect_packet.h"

#include <GL/gl.h>

using namespace k4a;
using namespace k4aviewer;

KFRDepthToPointcloudGPU::KFRDepthToPointcloudGPU(const k4a::calibration& k4a_calibration,
                                                 k4a_calibration_type_t calibrationType): _kfr(NULL) {
    k4a::image xyTable = GpuDepthToPointCloudConverter::GenerateXyTable(k4a_calibration, calibrationType);
    _converter.SetActiveXyTable(xyTable);
}

KFRDepthToPointcloudGPU::~KFRDepthToPointcloudGPU() {}

void KFRDepthToPointcloudGPU::setKinectFrameRecipient(KinectFrameRecipient* kfr) {
    _kfr = kfr;
}

void KFRDepthToPointcloudGPU::initializePacket(KinectPacket* kp) {
    kp->_mats["PointCloud"] = cv::Mat(512,512,CV_32FC4);
}

void KFRDepthToPointcloudGPU::receiveFrame(KinectPacket* kp) {
    k4a::image depth_raw = kp->_capture.get_depth_image();
    OpenGL::Texture texture;
    _converter.Convert(depth_raw, &texture);

    GLenum texture_width, texture_height;

    glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, (GLint*)&texture_width);
    glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_HEIGHT, (GLint*)&texture_height);
    unsigned char* texture_bytes = kp->_mats["PointCloud"].data;
    glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_UNSIGNED_BYTE, texture_bytes);

    if(_kfr) {
        _kfr->initializePacket(kp);
        _kfr->receiveFrame(kp);
    }
}