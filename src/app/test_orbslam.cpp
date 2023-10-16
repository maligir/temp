#include "System.h"
#include "Map.h"
#include "KeyFrame.h"
#include "kinect_playback.h"
#include "rosbag_playback.h"
#include "kinect_frame_recipient.h"
#include "kfr_capture_to_timestamp.h"
#include "kfr_capture_to_bgra.h"
#include "kfr_capture_to_depth.h"
#include "kfr_depth_to_pointcloud.h"
#include "rfr_get_frame_info.h"
#include "rfr_sync_image_frames.h"
#include "fr_bgra_to_rgb.h"
#include "fr_track_orb_slam.h"
#include "fr_depth_map_filter.h"
#include "fr_rgb_4k_to_720p.h"
#include "fr_depth_to_pointcloud.h"
#include "map_merger.h"
#include "mm_point_cloud_iou.h"
#include "fr_map_merging_handler.h"
#include "settings.h"

DevicePlayback* getK4Aplayer(Settings& settings) {
    std::string k4a_rec;
    float depth_min, depth_max;
    settings.getRequiredParameter("Depth.Min", depth_min);
    settings.getRequiredParameter("Depth.Max", depth_max);
    settings.getRequiredParameter("Record.Path", k4a_rec);

    KinectPlayback* kinect = new KinectPlayback(k4a_rec);
    const k4a::calibration* k4a_calibration = new k4a::calibration(kinect->GetCalibration());

    KFRCaptureToTimeStamp* kfrCaptureToTimeStamp = new KFRCaptureToTimeStamp();
    KFRCaptureToBGRA* kfrCaptureToBGRA = new KFRCaptureToBGRA();
    FRBGRAToRGB* frBGRAToRGB = new FRBGRAToRGB("BGRA4K", "RGB4K");
    KFRCaptureToDepth* kfrCaptureToDepth = new KFRCaptureToDepth(k4a_calibration);
    FRRGB4KTo720p* frRGB4KTo720p = new FRRGB4KTo720p();
    FRDepthMapFilter* frDepthMapFilter = new FRDepthMapFilter(depth_min, depth_max, "Depth720p");
    KFRDepthToPointcloud* kfrDepthToPointcloud = new KFRDepthToPointcloud(k4a_calibration);

    frDepthMapFilter->setFrameRecipient((FrameRecipient*)kfrDepthToPointcloud);
    frRGB4KTo720p->setFrameRecipient((FrameRecipient*)frDepthMapFilter);
    kfrCaptureToDepth->setFrameRecipient((FrameRecipient*)frRGB4KTo720p);
    frBGRAToRGB->setFrameRecipient((FrameRecipient*)kfrCaptureToDepth);
    kfrCaptureToBGRA->setFrameRecipient((FrameRecipient*)frBGRAToRGB);
    kfrCaptureToTimeStamp->setFrameRecipient((FrameRecipient*)kfrCaptureToBGRA);
    kinect->setFrameRecipient((FrameRecipient*)kfrCaptureToTimeStamp);

    return kinect;
}

DevicePlayback* getROSplayer(Settings& settings) {
    std::string rosbag_name;
    std::string img_info_topic;
    std::string img_topic;
    std::string depth_info_topic;
    std::string depth_topic;
    int img_width;
    int img_height;
    int depth_width;
    int depth_height;

    settings.getRequiredParameter("ROSBag.Path", rosbag_name);
    settings.getRequiredParameter("RGB.Camera.Info.Topic", img_info_topic);
    settings.getRequiredParameter("RGB.Camera.Image.Topic", img_topic);
    settings.getRequiredParameter("Depth.Camera.Info.Topic", depth_info_topic);
    settings.getRequiredParameter("Depth.Camera.Image.Topic", depth_topic);
    settings.getRequiredParameter("Image.Width", img_width);
    settings.getRequiredParameter("Image.Height", img_height);
    settings.getRequiredParameter("Depth.Width", depth_width);
    settings.getRequiredParameter("Depth.Height", depth_height);

    std::string img_key = "Image";
    std::string depth_key = "Depth";

     const std::vector<std::string> topics({img_info_topic,
                                            img_topic,
                                            depth_info_topic,
                                            depth_topic});

    // ROSPlayback* ros = NULL;
    ROSPlayback* ros = new ROSPlayback(rosbag_name, topics);

    ImageRes img_res(img_height, img_width, 3, 1);
    ImageRes depth_res(depth_height, depth_width, 1, 4);

    // RFRGetFrameInfo* get_img_info = new RFRGetFrameInfo(img_info_topic, img_key);
    // RFRSyncImageFrames* image_sync = new RFRSyncImageFrames(
    //     img_topic,
    //     depth_topic,
    //     img_key,
    //     depth_key,
    //     img_res,
    //     depth_res,
    //     CV_8UC3,
    //     CV_32FC1);
    // FRDepthToPointcloud* depth_to_pc = new FRDepthToPointcloud(depth_key);
    
    // image_sync->setFrameRecipient((FrameRecipient*)depth_to_pc);
    // get_img_info->setFrameRecipient((FrameRecipient*)image_sync);
    // ros->setFrameRecipient((FrameRecipient*)get_img_info);

    return ros;
}


void runORBSLAM(Settings& settings, ORB_SLAM3::System& SLAM) {
    std::chrono::microseconds start;
    std::chrono::microseconds end;

    DevicePlayback* playback;

    settings.readSensorSettings();
    std::string sensor;
    settings.getRequiredParameter("Sensor.Type", sensor);

    std::string img_key;
    std::string depth_key;


    float depth_min, depth_max;
    settings.getRequiredParameter("Depth.Min", depth_min);
    settings.getRequiredParameter("Depth.Max", depth_max);

    int s, e;
    bool isFound;
    s = settings.getParameter<int>("Recording.Start", isFound);
    if (!isFound) {
        s = 0;
    }
    start = std::chrono::microseconds(s);
    e = settings.getParameter<int>("Recording.End", isFound);
    if (!isFound) {
        e = std::numeric_limits<int>::max();
    }
    end = std::chrono::microseconds(e);

    if (sensor == std::string("K4A")) {
        std::cout << "getting K4A playback\n";
        playback = getK4Aplayer(settings);

        img_key = "RGB720p";
        depth_key = "Depth720p";
    } else if (sensor == std::string("ROS")) {
        playback = getROSplayer(settings);

        img_key = "Image";
        depth_key = "Depth";
    } else {
        std::cerr << "No implementation available for sensor type " << sensor << std::endl;
        exit(1);
    }

    settings.readMapMergingSettings();
    int nsamples;
    int max_unmerged;
    float step_decay;
    float max_radius;
    float thr_close;
    float lin_step_min;
    float ang_step_min;
    float lin_vel_max;
    float ang_vel_max;
    settings.getRequiredParameter("NSamples", nsamples);
    settings.getRequiredParameter("Unmerged.Frames.Max", max_unmerged);
    settings.getRequiredParameter("Step.Decay", step_decay);
    settings.getRequiredParameter("Radius.Max", max_radius);
    settings.getRequiredParameter("Threshold.Close", thr_close);
    settings.getRequiredParameter("Linear.Step.Min", lin_step_min);
    settings.getRequiredParameter("Angular.Step.Min", ang_step_min);
    settings.getRequiredParameter("Linear.Velocity.Max", lin_vel_max);
    settings.getRequiredParameter("Angular.Velocity.Max", ang_vel_max);

    MMPointCloudIOU mmPointCloudIOU(lin_vel_max,
                                    ang_vel_max,
                                    nsamples,
                                    step_decay,
                                    lin_step_min,
                                    ang_step_min,
                                    thr_close,
                                    max_radius,
                                    static_cast<uint16_t>(1e3*depth_min),
                                    static_cast<uint16_t>(1e3*depth_max),
                                    max_unmerged);

    playback->seekFrame(start);

    FRTrackORBSLAM frTrackORBSLAM(SLAM, img_key, depth_key);
    FRMapMergingHandler frmmHandler(SLAM, (MapMerger*)&mmPointCloudIOU);
    
    frTrackORBSLAM.setFrameRecipient((FrameRecipient*)&frmmHandler);
    playback->getLastFR()->setFrameRecipient((FrameRecipient*)&frTrackORBSLAM);

    playback->start();
    playback->seekFrame(start);

    std::cout << "starting playback at time: " << start.count() << std::endl;
    std::cout << "ending playback at time: " << end.count() << std::endl;
    std::cout << "timestamp currently set to: " << playback->getDataPacket().tframe.count() << std::endl;

    // while (!playback.isFinished() && count  < 1000) {
    std::cout << "beginning playback\n";
    while (!playback->isFinished() && playback->getDataPacket().tframe < end) {
        std::cout << "calling doOnce\n";
        playback->doOnce();
    }
    std::cout << "finished playback\n";

    delete playback;
}

int main(int argc, char**argv) {
    if (argc < 2) {
        std::cerr << "\nUsage: " << argv[0]
                  << " path_to_settings\n";
        return 1;
    }
    Settings settings(argv[1]);

    settings.readORBSettings();
    std::string orb_voc;
    std::string orb_set;
    settings.getRequiredParameter("ORB.Vocab", orb_voc);
    settings.getRequiredParameter("ORB.Settings", orb_set);

    ORB_SLAM3::System SLAM(orb_voc, orb_set, ORB_SLAM3::System::RGBD, true);
    runORBSLAM(settings, SLAM);

    // size_t start_time = 0;
    size_t start_time = 5947911; // starts moving
    // size_t start_time = 11614577; // starts rotating
    // size_t start_time = 24414577; // mid rotation - breaks @100000
    // size_t start_time = (23014577 + 23614577)/2;
    // size_t start_time = 23614577; // mid rotation - 
    // size_t start_time = 24814577; // mid rotation - breaks @10000 features, works @100000
    // size_t start_time = (24814577 + 24881244)/2; // mid rotation - test
    // size_t start_time = 24881244; // mid rotation - works
    // size_t start_time = 26881244; // finishes rotation
    // size_t start_time = 29481233; // after rotation

    // size_t end_time = 29481233; // after rotation
    // size_t end_time = 39481233; // after rotation

    std::string k4a_rec;
    settings.getRequiredParameter("Record.Path", k4a_rec);
    KinectPlayback playback(k4a_rec);
    playback.seekFrame(std::chrono::microseconds{start_time});
    k4a::calibration k4a_calibration = playback.GetCalibration();

    KFRCaptureToTimeStamp kfrCaptureToTimeStamp;
    KFRCaptureToBGRA kfrCaptureToBGRA;
    FRBGRAToRGB FRBGRAToRGB("BGRA4K", "RGB4K");
    KFRCaptureToDepth kfrCaptureToDepth(&k4a_calibration);
    FRDepthMapFilter frDepthMapFilter(0.25, 2.88, "Depth720p");
    FRRGB4KTo720p frRGB4KTo720p;
    FRTrackORBSLAM frTrackORBSLAM(SLAM, "RGB720p", "Depth720p");

    frRGB4KTo720p.setFrameRecipient((FrameRecipient*)&frTrackORBSLAM);
    frDepthMapFilter.setFrameRecipient((FrameRecipient*)&frRGB4KTo720p);
    kfrCaptureToDepth.setFrameRecipient((FrameRecipient*)&frDepthMapFilter);
    FRBGRAToRGB.setFrameRecipient((FrameRecipient*)&kfrCaptureToDepth);
    kfrCaptureToBGRA.setFrameRecipient((FrameRecipient*)&FRBGRAToRGB);
    kfrCaptureToTimeStamp.setFrameRecipient((FrameRecipient*)&kfrCaptureToBGRA);
    playback.setFrameRecipient((FrameRecipient*)&kfrCaptureToTimeStamp);

    playback.start();
    size_t count = 0;
    size_t tframe;
    while (!playback.isFinished() && count < 25) {
        count++;
        playback.doOnce();
        tframe = playback.getDataPacket().tframe.count();
        // std::cout << "processed frame timestamp: " << tframe << std::endl;
        // std::cout << "frame pose:\n" << playback.getDataPacket()._mats["Tcw"] << std::endl;
        // sleep(1);
    }
    std::cout << "last processed frame timestamp: " << tframe << std::endl;

    std::cout << "force create new map\n";
    SLAM.ForceTrackingLoss();

    std::cout << "jumping frames\n";
    playback.seekFrame(std::chrono::microseconds{11281244});
    Eigen::Matrix4f eig_trans;
    eig_trans.block<3,3>(0,0) = Eigen::AngleAxisf(-0.2373648, Eigen::Vector3f::UnitY()).matrix();
    eig_trans(0,3) = 0.0200366;
    eig_trans(1,3) = -0.0140674;
    eig_trans(2,3) = -1.1382;

    eig_trans(3,0) = 0;
    eig_trans(3,1) = 0;
    eig_trans(3,2) = 0;
    eig_trans(3,3) = 1;

    count = 0;
    while (!playback.isFinished()
        //    && tframe  < end_time) {
            && count  < 25) {
        count++;
        playback.doOnce();
        tframe = playback.getDataPacket().tframe.count();
        // std::cout << "processed frame timestamp: " << tframe << std::endl;
        // std::cout << "frame pose:\n" << playback.getDataPacket()._mats["Tcw"] << std::endl;
        // sleep(1);
    }

    const std::vector<ORB_SLAM3::Map*>& maps = SLAM.GetAllMaps();
    std::cout << "number of maps: " << maps.size() << std::endl;
    ORB_SLAM3::KeyFrame* adjusted(NULL);
    if (maps.size() > 1) {
        std::vector<ORB_SLAM3::KeyFrame*> kfs_m1 = maps[0]->GetAllKeyFrames();
        std::vector<ORB_SLAM3::KeyFrame*> kfs_m2 = maps[1]->GetAllKeyFrames();

        sort(kfs_m1.begin(),kfs_m1.end(),ORB_SLAM3::KeyFrame::lId);
        sort(kfs_m2.begin(),kfs_m2.end(),ORB_SLAM3::KeyFrame::lId);

        ORB_SLAM3::KeyFrame* m1_end = kfs_m1[kfs_m1.size()-1];
        ORB_SLAM3::KeyFrame* m2_begin = kfs_m2[0];

        std::cout << "last keyframe of first map: " << static_cast<size_t>(1e6*(m1_end->mTimeStamp)) << std::endl;
        std::cout << "first keyframe of second map: " << static_cast<size_t>(1e6*(m2_begin->mTimeStamp)) << std::endl;

        std::cout << "last keyframe first map pose:\n" << m1_end->GetPoseInverse().matrix() << std::endl;
        std::cout << "first keyframe second map pose:\n" << m2_begin->GetPoseInverse().matrix() << std::endl;

        Eigen::Matrix4f m1_end_pose = m1_end->GetPose().matrix();
        Eigen::Matrix4f m1e_to_m2b = eig_trans * m1_end_pose;
        std::cout << "transformation from last keykframe first map to first keyframe second map:\n" << m1e_to_m2b << std::endl;
        Sophus::SE3f sophus_trans(eig_trans.inverse());

        std::cout << "combining maps\n";
        SLAM.ForceMapMerge(m1_end, m2_begin, sophus_trans.cast<double>());
        std::cout << "pose after combination:\n" << m2_begin->GetPoseInverse().matrix() << std::endl;
        adjusted = m2_begin;
    }

    std::cout << "continuing video\n";
    count = 0;
    while (!playback.isFinished()
        //    && tframe  < end_time) {
            && count  < 3000) {
        count++;
        playback.doOnce();
        tframe = playback.getDataPacket().tframe.count();
        // if (adjusted) {
        //     std::cout << "adjusted pose  at timestamp:" << tframe << std::endl << adjusted->GetPoseInverse().matrix() << std::endl;
        // }
        // std::cout << "processed frame timestamp: " << tframe << std::endl;
        // std::cout << "frame pose:\n" << playback.getDataPacket()._mats["Tcw"] << std::endl;
        // sleep(1);
    }

    const std::vector<ORB_SLAM3::Map*>& maps2 = SLAM.GetAllMaps();
    std::cout << "number of maps: " << maps2.size() << std::endl;
    if (adjusted) {
        std::cout << "adjusted pose after run:\n" << adjusted->GetPoseInverse().matrix() << std::endl;
    }

    while(true) {
        sleep(1);
    }

}