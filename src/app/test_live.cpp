#include "System.h"
#include "kinect_wrapper.h"
#include "kinect_frame_recipient.h"
#include "kfr_capture_to_timestamp.h"
#include "kfr_capture_to_bgra.h"
#include "kfr_capture_to_depth.h"
#include "fr_bgra_to_rgb.h"
#include "fr_track_orb_slam.h"

#include <k4a/k4a.hpp>

int main(int argc, char**argv) {
    if (argc != 3) {
        std::cerr << "\nUsage: ./test_live path_to_orb_vocabulary path_to_orb_settings\n";
        return 1;
    }
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::RGBD,true);

    KinectWrapper kw(0);
    k4a::calibration& k4a_calibration = kw.GetCalibration();
    KFRCaptureToTimeStamp kfrCaptureToTimeStamp;
    KFRCaptureToBGRA kfrCaptureToBGRA;
    KFRCaptureToDepth kfrCaptureToDepth(&k4a_calibration);

    FRBGRAToRGB frBGRAToRGB("BGRA4K", "RGB4K");
    FRTrackORBSLAM frTrackORBSLAM(SLAM, "RGB4K", "Depth4K");
    
    frBGRAToRGB.setFrameRecipient((FrameRecipient*)&frTrackORBSLAM);
    kfrCaptureToDepth.setFrameRecipient((FrameRecipient*)&frBGRAToRGB);
    kfrCaptureToBGRA.setFrameRecipient((FrameRecipient*)&kfrCaptureToDepth);
    kfrCaptureToTimeStamp.setFrameRecipient((FrameRecipient*)&kfrCaptureToBGRA);
    kw.setFrameRecipient((FrameRecipient*)&kfrCaptureToTimeStamp);

    kw.start();
    while(true) {
        kw.doOnce();
    }

    return 0;
}