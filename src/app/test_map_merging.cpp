#include "System.h"
#include "Map.h"
#include "KeyFrame.h"
#include "kinect_playback.h"
#include "kinect_frame_recipient.h"
#include "kfr_capture_to_timestamp.h"
#include "kfr_capture_to_bgra.h"
#include "kfr_capture_to_depth.h"
#include "fr_bgra_to_rgb.h"
#include "fr_track_orb_slam.h"
#include "fr_depth_map_filter.h"
#include "fr_rgb_4k_to_720p.h"
#include "map_merger.h"
#include "fr_map_merging_handler.h"
#include "mm_point_cloud_iou.h"
#include "kfr_depth_to_pointcloud.h"

int main(int argc, char**argv) {
    if (argc != 4) {
        std::cerr << "\nUsage: ./play_video path_to_orb_vocabulary path_to_orb_settings path_to_k4a_recording\n";
        return 1;
    }
    const float depth_min = 0.25;
    const float depth_max = 2.88;

    // size_t start_time =    5947911; // starts moving
    size_t start_time =  657481233; // shortcut to map lost rotation
    // size_t end_time =   1059481233; // after hallway exit
    size_t end_time =   3059481233;

    KinectPlayback playback(argv[3]);
    playback.seekFrame(std::chrono::microseconds{start_time});
    k4a::calibration k4a_calibration = playback.GetCalibration();
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::RGBD,true);

    MMPointCloudIOU mmPointCloudIOU(0.075,
                                    0.05,
                                    10,
                                    0.5,
                                    0.001,
                                    0.001/depth_max,
                                    0.005,
                                    3.0,
                                    static_cast<uint16_t>(1e3*depth_min),
                                    static_cast<uint16_t>(1e3*depth_max),
                                    300);
    // mmPointCloudIOU.setDebugFlag(true);

    KFRCaptureToTimeStamp kfrCaptureToTimeStamp;
    KFRCaptureToBGRA kfrCaptureToBGRA;
    FRBGRAToRGB FRBGRAToRGB("BGRA4K", "RGB4K");
    KFRCaptureToDepth kfrCaptureToDepth(&k4a_calibration);
    FRDepthMapFilter frDepthMapFilter(depth_min, depth_max, "Depth720p");
    KFRDepthToPointcloud kfrDepthToPointcloud(&k4a_calibration);
    FRRGB4KTo720p frRGB4KTo720p;
    FRTrackORBSLAM frTrackORBSLAM(SLAM, "RGB720p", "Depth720p");
    FRMapMergingHandler frmmHandler(SLAM, (MapMerger*)&mmPointCloudIOU);

    std::cout << "kfrs decalred\n";

    frTrackORBSLAM.setFrameRecipient((FrameRecipient*)&frmmHandler);
    frRGB4KTo720p.setFrameRecipient((FrameRecipient*)&frTrackORBSLAM);
    kfrDepthToPointcloud.setFrameRecipient((FrameRecipient*)&frRGB4KTo720p);
    frDepthMapFilter.setFrameRecipient((FrameRecipient*)&kfrDepthToPointcloud);
    kfrCaptureToDepth.setFrameRecipient((FrameRecipient*)&frDepthMapFilter);
    FRBGRAToRGB.setFrameRecipient((FrameRecipient*)&kfrCaptureToDepth);
    kfrCaptureToBGRA.setFrameRecipient((FrameRecipient*)&FRBGRAToRGB);
    kfrCaptureToTimeStamp.setFrameRecipient((FrameRecipient*)&kfrCaptureToBGRA);
    playback.setFrameRecipient((FrameRecipient*)&kfrCaptureToTimeStamp);

    std::cout << "call chain built\n";

    playback.start();
    std::cout << "playback started\n";
    size_t tframe = 0;
    size_t count = 0;
    while (!playback.isFinished()
           && tframe  < end_time) {
        count++;
        // std::cout << "on loop: " << count << std::endl;
        playback.doOnce();
        // std::cout << "processed frame\n";
        tframe = playback.getDataPacket().tframe.count();
        // std::cout << "processed frame timestamp: " << tframe << std::endl;
        // std::cout << "frame pose:\n" << playback.getDataPacket()._mats["Tcw"] << std::endl;
        // sleep(1);
    }

    std::cout << "finished\n";
    while(true) {
        sleep(1);
    }

}