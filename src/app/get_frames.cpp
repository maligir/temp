#include "kinect_playback.h"
#include "rosbag_playback.h"
#include "fr_bgra_to_rgb.h"
#include "fr_depth_map_filter.h"
#include "fr_rgb_4k_to_720p.h"
#include "fr_record_frames.h"
#include "kinect_frame_recipient.h"
#include "kfr_capture_to_timestamp.h"
#include "kfr_capture_to_bgra.h"
#include "fr_april_tag.h"
#include "fr_apriltag_pose.h"
#include "kfr_capture_to_depth.h"
#include "rfr_get_frame_info.h"
#include "rfr_sync_image_frames.h"
#include "device_playback.h"
#include "settings.h"


#include <k4a/k4a.hpp>
#include <unistd.h>

#include <boost/filesystem.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/dnn.hpp>

#include <fstream>
#include <string>

void getK4AFrames(Settings& settings) {
  std::string img_key = "RGB720p";
  std::string depth_key = "Depth720p";
  std::string trial_dir;
  settings.getRequiredParameter("Save.Path", trial_dir);

  float depth_min, depth_max;
  settings.getRequiredParameter("Depth.Min", depth_min);
  settings.getRequiredParameter("Depth.Max", depth_max);

  std::string k4a_rec;
  settings.getRequiredParameter("Record.Path", k4a_rec);

  settings.readAprilTagSettings();
  float tagH, tagW;
  settings.getRequiredParameter("AprilTag.Width", tagW);
  settings.getRequiredParameter("AprilTag.Height", tagH);

  AprilTagManager tagManager(tagW, tagH);

  KinectPlayback kinect(k4a_rec);
  const k4a::calibration k4a_calibration(kinect.GetCalibration());

  KFRCaptureToTimeStamp kfrCaptureToTimeStamp;
  KFRCaptureToBGRA kfrCaptureToBGRA;
  FRBGRAToRGB frBGRAToRGB("BGRA4K", "RGB4K");
  KFRCaptureToDepth kfrCaptureToDepth(&k4a_calibration);
  FRRGB4KTo720p frRGB4KTo720p;
  FRDepthMapFilter frDepthMapFilter(depth_min, depth_max, depth_key);
  FRRecordFrames frRecordFrames(img_key, depth_key, trial_dir);
  FRAprilTag kfrAprilTag(&tagManager, "RGB4K", "Depth4K");
  FRAprilTagPose kfrApriltagPose;

  kfrApriltagPose.setFrameRecipient((FrameRecipient*)&frRecordFrames);
  kfrAprilTag.setFrameRecipient((FrameRecipient*)&kfrApriltagPose);
  frDepthMapFilter.setFrameRecipient((FrameRecipient*)&kfrAprilTag);
  frRGB4KTo720p.setFrameRecipient((FrameRecipient*)&frDepthMapFilter);
  kfrCaptureToDepth.setFrameRecipient((FrameRecipient*)&frRGB4KTo720p);
  frBGRAToRGB.setFrameRecipient((FrameRecipient*)&kfrCaptureToDepth);
  kfrCaptureToBGRA.setFrameRecipient((FrameRecipient*)&frBGRAToRGB);
  kfrCaptureToTimeStamp.setFrameRecipient((FrameRecipient*)&kfrCaptureToBGRA);
  kinect.setFrameRecipient((FrameRecipient*)&kfrCaptureToTimeStamp);

  kinect.start();

  while (!kinect.isFinished()) {
    kinect.doOnce();
  }
  tagManager.solve();

  std::ofstream oframes;
  oframes.open(trial_dir+"aprilTag_frames.txt");
  std::map<size_t, AprilFrame*>& aFrames = tagManager.getFrames();
  for (auto frame : aFrames) {
    double* params = frame.second->getParams();
    std::string line = std::to_string(frame.first);
    for (uint8_t i = 0; i < 7; i++) {
      line += (", " + std::to_string(params[i]));
    }
    oframes << line << std::endl;
  }
  oframes.close();

  std::ofstream oTags;
  oTags.open(trial_dir+"aprilTag_tags.txt");
  std::map<size_t, AprilTag*>& aTags = tagManager.getTags();
  for (auto tag : aTags) {
    double* params = tag.second->getParams();
    std::string line = std::to_string(tag.first);
    for (uint8_t i = 0; i < 7; i++) {
      line += (", " + std::to_string(params[i]));
    }
    oTags << line << std::endl;
  }
  oTags.close();
}

void getROSFrames(Settings& settings) {
  std::string img_key = "Image";
  std::string depth_key = "Depth";
  std::string trial_dir;
  settings.getRequiredParameter("Save.Path", trial_dir);

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

  std::vector<std::string> topics({img_info_topic,
                                  img_topic,
                                  depth_info_topic,
                                  depth_topic});
    
  ImageRes img_res(img_height, img_width, 3, 1);
  ImageRes depth_res(depth_height, depth_width, 1, 4);

  ROSPlayback ros(rosbag_name, topics);

  RFRGetFrameInfo get_img_info(img_info_topic, img_key);
  RFRSyncImageFrames image_sync(img_topic,
                                depth_topic,
                                img_key,
                                depth_key,
                                img_res,
                                depth_res,
                                CV_8UC3,
                                CV_32FC1);
  FRDepthMapFilter frDepthMapFilter(depth_min, depth_max, depth_key);
  FRRecordFrames frRecordFrames(img_key, depth_key, trial_dir);

  frDepthMapFilter.setFrameRecipient((FrameRecipient*)&frRecordFrames);
  image_sync.setFrameRecipient((FrameRecipient*)&frDepthMapFilter);
  get_img_info.setFrameRecipient((FrameRecipient*)&image_sync);
  ros.setFrameRecipient((FrameRecipient*)&get_img_info);

  ros.start();

  while (!ros.isFinished()) {
    ros.doOnce();
  }
}

int main(int argc, char**argv) {
  if (argc < 2) {
    std::cerr << "\nUsage: " << argv[0]
              << " path_to_settings\n";
    return 1;
  }

  DevicePlayback* playback;
  Settings settings(argv[1]);
  settings.readSensorSettings();
  std::string sensor;
  settings.getRequiredParameter("Sensor.Type", sensor);

  if (sensor == "K4A") {
    getK4AFrames(settings);
  } else if (sensor == "ROS") {
    getROSFrames(settings);
  } else {
    std::cerr << "No implementation available for sensor type " << sensor << std::endl;
    return 1;
  }
}