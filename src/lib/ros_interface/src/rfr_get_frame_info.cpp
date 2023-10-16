#include "rfr_get_frame_info.h"

#include <sensor_msgs/CameraInfo.h>

RFRGetFrameInfo::RFRGetFrameInfo(const std::string& topic,
                                 const std::string& cam_name):
    _topic(topic)
    , _cam_name(cam_name)
    , _isInfoSet(false)
    , _Kname(_cam_name+"_K")
    , _invKname(_cam_name+"_Kinv") {}

void RFRGetFrameInfo::initializePacket(ROSPacket *rp) {
  rp->_eigs[_Kname] = Eigen::MatrixXf::Identity(4,4);
  rp->_eigs[_invKname] = Eigen::MatrixXf::Identity(4,4);

  if (_fr) {
    _fr->initializePacket(rp);
  }
}

void RFRGetFrameInfo::receiveFrame(ROSPacket *rp) {
  sensor_msgs::CameraInfoPtr info = NULL;
  
  if (!_isInfoSet && rp->_msg->getTopic() == _topic) {
    info = rp->_msg->instantiate<sensor_msgs::CameraInfo>();
  }

  if (info != NULL) {
    Eigen::Map<Eigen::Matrix<double,3,3,Eigen::RowMajor>> M(info->K.data());
    rp->_eigs[_Kname].block<3,3>(0,0) = M.cast<float>();
    rp->_eigs[_invKname] = rp->_eigs[_Kname].inverse();
  }

  if (_fr) {
    _fr->receiveFrame(rp);
  }

}