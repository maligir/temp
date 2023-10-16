#include "fr_map_merging_handler.h"

#include "System.h"
#include "data_packet.h"

FRMapMergingHandler::FRMapMergingHandler(ORB_SLAM3::System& ORB_SLAM,
                                           MapMerger* map_merger):
    _ORB_SLAM(ORB_SLAM)
    , _map_merger(map_merger) {}

FRMapMergingHandler::~FRMapMergingHandler() {}

void FRMapMergingHandler::initializePacket(DataPacket* dp) {
    if (_fr) {
        _fr->initializePacket(dp);
    }
}

void FRMapMergingHandler::receiveFrame(DataPacket* dp) {
    uint8_t orb_state = _ORB_SLAM.GetCurrentState();
    const std::vector<ORB_SLAM3::Map*>& maps = _ORB_SLAM.GetAllMaps();

    if (orb_state == ORB_SLAM3::Tracking::OK && maps.size() == 1) {
        _map_merger->updateTracking(dp);
    }

    if (orb_state != ORB_SLAM3::Tracking::OK && maps.size() == 1) {
        _map_merger->updateLost(dp);
    }

    if (orb_state == ORB_SLAM3::Tracking::OK && maps.size() > 1) {
        _map_merger->updateTrackingUnmerged(dp);
    }

    if (orb_state != ORB_SLAM3::Tracking::OK && maps.size() > 1) {
        std::cout << "lost with: " << maps.size() << " unmerged maps.\n";
        _map_merger->updateLostUnmerged(dp);
    }

    if (_map_merger->isMergeReady()) {
        std::vector<ORB_SLAM3::KeyFrame*> kfs_m1 = maps[0]->GetAllKeyFrames();
        std::vector<ORB_SLAM3::KeyFrame*> kfs_m2 = maps[1]->GetAllKeyFrames();

        sort(kfs_m1.begin(),kfs_m1.end(),ORB_SLAM3::KeyFrame::lId);
        sort(kfs_m2.begin(),kfs_m2.end(),ORB_SLAM3::KeyFrame::lId);

        ORB_SLAM3::KeyFrame* m1_end = kfs_m1[kfs_m1.size()-1];
        ORB_SLAM3::KeyFrame* m2_begin = kfs_m2[0];

        Eigen::Matrix4f eig_trans = _map_merger->getTransform();
        Sophus::SE3f sophus_trans(eig_trans);
        _ORB_SLAM.ForceMapMerge(m1_end, m2_begin, sophus_trans.cast<double>());
    }

    if (_fr) {
        _fr->receiveFrame(dp);
    }
}