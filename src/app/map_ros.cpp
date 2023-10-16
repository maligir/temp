#include "System.h"
#include "Map.h"
#include "KeyFrame.h"
#include "rosbag_playback.h"
#include "index_reader_playback.h"
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
#include "orbslam_utils.h"
#include "octmap_utils.h"
#include "settings.h"

#include <eigen3/Eigen/Core>
#include <unistd.h>

#include <boost/filesystem.hpp>

#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/dnn.hpp>

void runROSORBSLAM(Settings& settings, ORB_SLAM3::System* SLAM) {
  std::chrono::microseconds start;
  std::chrono::microseconds end;

  std::string sensor;
  std::string save_dir;
  settings.getRequiredParameter("Sensor.Type", sensor);
  settings.getRequiredParameter("Save.Path", save_dir);
  std::string img_index = save_dir + "img_index.txt";

  std::string img_key = "Image";
  std::string depth_key = "Depth";

  float depth_min, depth_max;
  settings.getRequiredParameter("Depth.Min", depth_min);
  settings.getRequiredParameter("Depth.Max", depth_max);

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

  long s, e;
  bool isFound;
  s = settings.getParameter<int>("ROSBag.Start", isFound);
  if (!isFound) {
    s = 0;
  }
  start = std::chrono::microseconds(s);
  e = settings.getParameter<int>("ROSBag.End", isFound);
  if (!isFound) {
    e = LONG_MAX;
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
    
  ImageRes img_res(img_height, img_width, 3, 1);
  ImageRes depth_res(depth_height, depth_width, 1, 4);

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

  // Because ROSPlayback appears incompatible with FRTrackORBSLAM,
  // we use index reader to read data after extraction instead
  // std::vector<std::string> topics({img_info_topic,
  //                                 img_topic,
  //                                 depth_info_topic,
  //                                 depth_topic});
  // ROSPlayback ros(rosbag_name, topics);

  // this code replaces ROSPlayback functionality
  IndexReaderPlayback index(save_dir);
  std::string prefix = "RGB.Camera.";
  settings.readCameraIntrinsics("RGB");
  Eigen::Matrix4f K = Eigen::Matrix4f::Identity();
  settings.getRequiredParameter(prefix+"AlphaX", K(0,0));
  settings.getRequiredParameter(prefix+"AlphaY", K(1,1));
  settings.getRequiredParameter(prefix+"Gamma", K(0,1));
  settings.getRequiredParameter(prefix+"u0", K(0,2));
  settings.getRequiredParameter(prefix+"v0", K(1,2));
  index.getDataPacket()._eigs[img_key+"_K"] = K;
  index.getDataPacket()._eigs[img_key+"_Kinv"] = K.inverse();
  index.getDataPacket()._mats[img_key] = cv::Mat(img_height, img_width, CV_8UC3);
  index.getDataPacket()._eigs[depth_key+"_K"] = K;
  index.getDataPacket()._eigs[depth_key+"_Kinv"] = K.inverse();
  index.getDataPacket()._mats[depth_key] = cv::Mat(depth_height, depth_width, CV_16UC1);

  // commented out for ROSPlayback
  // RFRGetFrameInfo get_img_info(img_info_topic, img_key);
  // RFRSyncImageFrames image_sync(img_topic,
  //                               depth_topic,
  //                               img_key,
  //                               depth_key,
  //                               img_res,
  //                               depth_res,
  //                               CV_8UC3,
  //                               CV_32FC1);
  FRDepthMapFilter frDepthMapFilter(depth_min, depth_max, depth_key);
  FRDepthToPointcloud frDepthToPC(depth_key);
  FRTrackORBSLAM frTrackORBSLAM(*SLAM, img_key, depth_key);
  FRMapMergingHandler frmmHandler(*SLAM, (MapMerger*)&mmPointCloudIOU);

  frTrackORBSLAM.setFrameRecipient((FrameRecipient*)&frmmHandler);
  frDepthToPC.setFrameRecipient((FrameRecipient*)&frTrackORBSLAM);
  frDepthMapFilter.setFrameRecipient((FrameRecipient*)&frDepthToPC);
  // commented out for ROSPlayback
  // image_sync.setFrameRecipient((FrameRecipient*)&depth_to_pc);
  // get_img_info.setFrameRecipient((FrameRecipient*)&image_sync);
  // ros.setFrameRecipient((FrameRecipient*)&get_img_info);
  index.setFrameRecipient((FrameRecipient*)&frDepthMapFilter);

  index.start();
  index.getDataPacket().tframe = std::chrono::microseconds(0);
  index.seekFrame(start);

  std::cout << "Ending Time: " << end.count() << std::endl;
  std::cout << "Running SLAM\n";
  while (!index.isFinished() && index.getDataPacket().tframe < end) {
    index.doOnce();
  }
  std::cout << "Finished building orbslam map\n";
}

void saveMapROS(Settings& settings, ORB_SLAM3::System* SLAM, std::string tfile) {
  std::chrono::microseconds start;
  std::chrono::microseconds end;

  std::string img_key;
  std::string depth_key;

  float depth_min, depth_max;
  settings.getRequiredParameter("Depth.Min", depth_min);
  settings.getRequiredParameter("Depth.Max", depth_max);

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

  ImageRes img_res(img_height, img_width, 3, 1);
  ImageRes depth_res(depth_height, depth_width, 1, 4);

  int s, e;
  bool isFound;
  s = settings.getParameter<int>("ROSBag.Start", isFound);
  if (!isFound) {
      s = 0;
  }
  start = std::chrono::microseconds(s);
  e = settings.getParameter<int>("ROSBag.End", isFound);
  if (!isFound) {
      e = std::numeric_limits<int>::max();
  }
  end = std::chrono::microseconds(e);

  img_key = "Image";
  depth_key = "Depth";

// Because ROSPlayback appears incompatible with FRTrackORBSLAM,
  // we use index reader to read data after extraction instead
  // std::vector<std::string> topics({img_info_topic,
  //                                 img_topic,
  //                                 depth_info_topic,
  //                                 depth_topic});
  // ROSPlayback ros(rosbag_name, topics);

  // this code replaces ROSPlayback functionality
  std::string save_dir;
  settings.getRequiredParameter("Save.Path", save_dir);
  IndexReaderPlayback index(save_dir);
  std::string prefix = "RGB.Camera.";
  settings.readCameraIntrinsics("RGB");
  Eigen::Matrix4f K = Eigen::Matrix4f::Identity();
  settings.getRequiredParameter(prefix+"AlphaX", K(0,0));
  settings.getRequiredParameter(prefix+"AlphaY", K(1,1));
  settings.getRequiredParameter(prefix+"Gamma", K(0,1));
  settings.getRequiredParameter(prefix+"u0", K(0,2));
  settings.getRequiredParameter(prefix+"v0", K(1,2));
  index.getDataPacket()._eigs[img_key+"_K"] = K;
  index.getDataPacket()._eigs[img_key+"_Kinv"] = K.inverse();
  index.getDataPacket()._mats[img_key] = cv::Mat(img_height, img_width, CV_8UC3);
  index.getDataPacket()._eigs[depth_key+"_K"] = K;
  index.getDataPacket()._eigs[depth_key+"_Kinv"] = K.inverse();
  index.getDataPacket()._mats[depth_key] = cv::Mat(depth_height, depth_width, CV_16UC1);

  // commented out for ROSPlayback
  // RFRGetFrameInfo get_img_info(img_info_topic, img_key);
  // RFRSyncImageFrames image_sync(img_topic,
  //                               depth_topic,
  //                               img_key,
  //                               depth_key,
  //                               img_res,
  //                               depth_res,
  //                               CV_8UC3,
  //                               CV_32FC1);
  FRDepthMapFilter frDepthMapFilter(depth_min, depth_max, depth_key);
  FRDepthToPointcloud frDepthToPC(depth_key);
  FRTrackORBSLAM frTrackORBSLAM(*SLAM, img_key, depth_key);

  frDepthToPC.setFrameRecipient((FrameRecipient*)&frTrackORBSLAM);
  frDepthMapFilter.setFrameRecipient((FrameRecipient*)&frDepthToPC);
  // commented out for ROSPlayback
  // image_sync.setFrameRecipient((FrameRecipient*)&depth_to_pc);
  // get_img_info.setFrameRecipient((FrameRecipient*)&image_sync);
  // ros.setFrameRecipient((FrameRecipient*)&get_img_info);
  index.setFrameRecipient((FrameRecipient*)&frDepthMapFilter);

  index.start();
  index.getDataPacket().tframe = std::chrono::microseconds(0);
  index.seekFrame(start);

  SLAM->ActivateLocalizationMode();

  std::ofstream ftraj;
  ftraj.open(tfile);
  size_t count = 0;
  while (!index.isFinished() && index.getDataPacket().tframe < end) {
    count++;
    index.doOnce();
    Eigen::MatrixXf pose = index.getDataPacket()._eigs["Tcw"].inverse();
    if (pose.rows() < 4 || pose.cols() < 4) {
        continue;
    }
    Eigen::Vector3f t = pose.block<3,1>(0,3);
    Eigen::Quaternionf q(pose.block<3,3>(0,0));
    ftraj << count << ","
        << setprecision(9)
        << 1e-6*static_cast<float>(index.getDataPacket().tframe.count()) << ","
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
  if (sensor != std::string("ROS")) {
    std::cout << "Invalid sensor type " << sensor << ", expected ROS sensor\n";
    return 1;
  }

  settings.readORBSettings();
  std::string orb_voc;
  std::string orb_set;
  settings.getRequiredParameter("ORB.Vocab", orb_voc);
  settings.getRequiredParameter("ORB.Settings", orb_set);

  ORB_SLAM3::System SLAM(orb_voc, orb_set, ORB_SLAM3::System::RGBD, true);
  runROSORBSLAM(settings, &SLAM);

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
    saveMapROS(settings, &SLAM, tfile);
  }

  SLAM.Shutdown();

  std::cout << "finished\n";
}