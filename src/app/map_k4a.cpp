#include "System.h"
#include "Map.h"
#include "KeyFrame.h"
#include "kinect_playback.h"
#include "fr_bgra_to_rgb.h"
#include "fr_track_orb_slam.h"
#include "fr_depth_map_filter.h"
#include "fr_rgb_4k_to_720p.h"
#include "kinect_frame_recipient.h"
#include "kfr_capture_to_timestamp.h"
#include "kfr_capture_to_bgra.h"
#include "kfr_capture_to_depth.h"
#include "kfr_depth_to_pointcloud.h"
#include "map_merger.h"
#include "mm_point_cloud_iou.h"
#include "fr_map_merging_handler.h"
#include "orbslam_utils.h"
#include "octmap_utils.h"
#include "settings.h"

#include <k4a/k4a.hpp>
#include <eigen3/Eigen/Core>
#include <unistd.h>

#include <boost/filesystem.hpp>

#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/dnn.hpp>

inline void runK4AORBSLAM(Settings& settings, ORB_SLAM3::System* SLAM) {
  std::chrono::microseconds start;
  std::chrono::microseconds end;

  std::string sensor;
  settings.getRequiredParameter("Sensor.Type", sensor);

  std::string img_key = "RGB720p";
  std::string depth_key = "Depth720p";

  float depth_min, depth_max;
  settings.getRequiredParameter("Depth.Min", depth_min);
  settings.getRequiredParameter("Depth.Max", depth_max);

  std::string k4a_rec;
  settings.getRequiredParameter("Record.Path", k4a_rec);

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

  KinectPlayback kinect(k4a_rec);
  const k4a::calibration k4a_calibration(kinect.GetCalibration());

  KFRCaptureToTimeStamp kfrCaptureToTimeStamp;
  KFRCaptureToBGRA kfrCaptureToBGRA;
  FRBGRAToRGB frBGRAToRGB("BGRA4K", "RGB4K");
  KFRCaptureToDepth kfrCaptureToDepth(&k4a_calibration);
  FRRGB4KTo720p frRGB4KTo720p;
  FRDepthMapFilter frDepthMapFilter(depth_min, depth_max, depth_key);
  KFRDepthToPointcloud kfrDepthToPointcloud(&k4a_calibration);
  FRTrackORBSLAM frTrackORBSLAM(*SLAM, img_key, depth_key);
  FRMapMergingHandler frmmHandler(*SLAM, (MapMerger*)&mmPointCloudIOU);

  frTrackORBSLAM.setFrameRecipient((FrameRecipient*)&frmmHandler);
  kfrDepthToPointcloud.setFrameRecipient((FrameRecipient*)&frTrackORBSLAM);
  frDepthMapFilter.setFrameRecipient((FrameRecipient*)&kfrDepthToPointcloud);
  frRGB4KTo720p.setFrameRecipient((FrameRecipient*)&frDepthMapFilter);
  kfrCaptureToDepth.setFrameRecipient((FrameRecipient*)&frRGB4KTo720p);
  frBGRAToRGB.setFrameRecipient((FrameRecipient*)&kfrCaptureToDepth);
  kfrCaptureToBGRA.setFrameRecipient((FrameRecipient*)&frBGRAToRGB);
  kfrCaptureToTimeStamp.setFrameRecipient((FrameRecipient*)&kfrCaptureToBGRA);
  kinect.setFrameRecipient((FrameRecipient*)&kfrCaptureToTimeStamp);

  kinect.start();
  kinect.getDataPacket().tframe = std::chrono::microseconds(0);
  kinect.seekFrame(start);

  while (!kinect.isFinished() && kinect.getDataPacket().tframe < end) {
    kinect.doOnce();
  }
}

inline void saveMapK4A(Settings& settings, ORB_SLAM3::System* SLAM, std::string tfile) {
  std::chrono::microseconds start;
  std::chrono::microseconds end;

  std::string img_key = "RGB720p";
  std::string depth_key = "Depth720p";

  float depth_min, depth_max;
  settings.getRequiredParameter("Depth.Min", depth_min);
  settings.getRequiredParameter("Depth.Max", depth_max);

  std::string k4a_rec;
  settings.getRequiredParameter("Record.Path", k4a_rec);

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

  KinectPlayback kinect(k4a_rec);
  k4a::calibration k4a_calibration(kinect.GetCalibration());

  KFRCaptureToTimeStamp kfrCaptureToTimeStamp;
  KFRCaptureToBGRA kfrCaptureToBGRA;
  FRBGRAToRGB frBGRAToRGB("BGRA4K", "RGB4K");
  KFRCaptureToDepth kfrCaptureToDepth(&k4a_calibration);
  FRRGB4KTo720p frRGB4KTo720p;
  FRDepthMapFilter frDepthMapFilter(depth_min, depth_max, depth_key);
  FRTrackORBSLAM frTrackORBSLAM(*SLAM, img_key, depth_key);

  frDepthMapFilter.setFrameRecipient((FrameRecipient*)&frTrackORBSLAM);
  frRGB4KTo720p.setFrameRecipient((FrameRecipient*)&frDepthMapFilter);
  kfrCaptureToDepth.setFrameRecipient((FrameRecipient*)&frRGB4KTo720p);
  frBGRAToRGB.setFrameRecipient((FrameRecipient*)&kfrCaptureToDepth);
  kfrCaptureToBGRA.setFrameRecipient((FrameRecipient*)&frBGRAToRGB);
  kfrCaptureToTimeStamp.setFrameRecipient((FrameRecipient*)&kfrCaptureToBGRA);
  kinect.setFrameRecipient((FrameRecipient*)&kfrCaptureToTimeStamp);

  kinect.start();
  kinect.getDataPacket().tframe = std::chrono::microseconds(0);
  kinect.seekFrame(start);

  SLAM->ActivateLocalizationMode();

  std::ofstream ftraj;
  ftraj.open(tfile);
  size_t count = 0;
  while (!kinect.isFinished() && kinect.getDataPacket().tframe < end) {
    count++;
    kinect.doOnce();
    Eigen::MatrixXf pose = kinect.getDataPacket()._eigs["Tcw"].inverse();
    if (pose.rows() < 4 || pose.cols() < 4) {
      continue;
    }
    Eigen::Vector3f t = pose.block<3,1>(0,3);
    Eigen::Quaternionf q(pose.block<3,3>(0,0));
    ftraj << count << ","
          << setprecision(9)
          << 1e-6*static_cast<float>(kinect.getDataPacket().tframe.count()) << ","
          << t(0) << ","
          << t(1) << ","
          << t(2) << ","
          << q.x() << ","
          << q.y() << ","
          << q.z() << ","
          << q.w() << "\n";
  }
  ftraj.close();
}

int main(int argc, char**argv) {
  if (argc < 2) {
    std::cerr << "\nUsage: " << argv[0]
              << " path_to_settings\n";
    return 1;
  }
  Settings settings(argv[1]);
  settings.readSensorSettings();

  std::string sensor;
  settings.getRequiredParameter("Sensor.Type", sensor);
  if (sensor != std::string("K4A")) {
    std::cout << "Invalid sensor type " << sensor << ", expected K4A sensor\n";
    return 1;
  }

  settings.readORBSettings();
  std::string orb_voc;
  std::string orb_set;
  settings.getRequiredParameter("ORB.Vocab", orb_voc);
  settings.getRequiredParameter("ORB.Settings", orb_set);

  ORB_SLAM3::System SLAM(orb_voc, orb_set, ORB_SLAM3::System::RGBD, true);
  runK4AORBSLAM(settings, &SLAM);

  const std::vector<ORB_SLAM3::Map*>& maps = SLAM.GetAllMaps();
  std::cout << "There are " << maps.size() << " maps to process\n";

  std::string trial_dir;
  settings.getRequiredParameter("Save.Path", trial_dir);
  boost::filesystem::create_directories(trial_dir.c_str());

  for (size_t i = 0; i < maps.size(); i++) {
    std::vector<ORB_SLAM3::KeyFrame*> kfs = maps[i]->GetAllKeyFrames();
    sort(kfs.begin(),kfs.end(),ORB_SLAM3::KeyFrame::lId);
    const std::vector<ORB_SLAM3::MapPoint*>& mps = maps[i]->GetAllMapPoints();
    std::string map_dir = trial_dir + "/map" + std::to_string(i+1);
    boost::filesystem::create_directories(map_dir);
    std::cout << "map " << i+1 << " has " << kfs.size() << " keyframes and " << mps.size() << " map points\n";
    std::string mfile = map_dir + "/orb_map_points.csv";
    std::string kfile = map_dir + "/keyframe_trajectory.csv";
    std::string ofile = map_dir + "/octomap.bt";
    std::string fmap = map_dir + "/occupation.csv";
    std::string fplacs = map_dir + "/placards.csv";
    SaveMapToFile(mfile, mps);
    SaveKeyframeTrajectoryToFile(kfile, kfs);
  }

  if (maps.size() == 1) {
    std::string tfile = trial_dir + "/full_trajectory.csv\n";
    saveMapK4A(settings, &SLAM, tfile);
  }

  SLAM.Shutdown();

  std::cout << "finished\n";
}