#include "fr_track_orb_slam.h"
#include "data_packet.h"

using namespace ORB_SLAM3;
using namespace cv;

FRTrackORBSLAM::FRTrackORBSLAM(ORB_SLAM3::System& SLAM,
                               const std::string& rgb_name,
                               const std::string& depth_name):
    _SLAM(SLAM)
    , _rgb_name(rgb_name)
    , _depth_name(depth_name) {}

void FRTrackORBSLAM::initializePacket(DataPacket *dp) {
    if (_fr) {
        _fr->initializePacket(dp);
    }
}

void FRTrackORBSLAM::receiveFrame(DataPacket *dp) {
    cv::Mat rgb = dp->_mats[_rgb_name];
    cv::Mat depth = dp->_mats[_depth_name];

    // cv::imshow("rgb", rgb);
    // cv::waitKey(0);

    Sophus::SE3f Tcw = _SLAM.TrackRGBD(rgb,depth,1e-6*static_cast<double>(dp->tframe.count()));
    dp->_eigs["Tcw"] = Tcw.matrix();
    dp->frameID = _SLAM.GetCurrentFrameID();
    dp->mapID = _SLAM.GetCurrentMapID();
    if (_fr) {
        _fr->receiveFrame(dp);
    }
}