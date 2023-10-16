#include "kinect_playback.h"
#include "kinect_frame_recipient.h"
#include "kfr_capture_to_timestamp.h"
#include "kfr_capture_to_bgra.h"
#include "kfr_capture_to_depth.h"
#include "fr_bgra_to_rgb.h"
#include "fr_rgb_4k_to_720p.h"
#include "fr_get_bounding_boxes.h"
#include "fr_extract_placard.h"
#include "kfr_depth_to_pointcloud.h"
#include "yolov5_observer.h"

#include <k4a/k4a.hpp>
#include <eigen3/Eigen/Core>
#include <unistd.h>

#include <boost/filesystem.hpp>

#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>
#include <tesseract/baseapi.h>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/dnn.hpp>

void parse_line(std::string& line, std::vector<std::chrono::microseconds>& times) {
  std::string delimiter = ",";

  size_t pos = 0;
  std::string token;
  while ((pos = line.find(delimiter)) != std::string::npos) {
      token = line.substr(0, pos);
      size_t time = std::stoul(token);
      std::chrono::microseconds tstamp(time);
      times.push_back(tstamp);
      line.erase(0, pos + delimiter.length());
  }
  size_t time = std::stoul(line);
  std::chrono::microseconds tstamp(time);
  times.push_back(tstamp);
}

int main(int argc, char**argv) {
  std::string video = "videos/segbot_v4_run.mkv";
  std::string yolo_onnx = "config/yolov5_best.onnx";
  std::string trial_dir = "data/trial-1";
  std::string east_net_f = "config/frozen_east_text_detection.pb";
  std::string tess_dir = "third_party/tesseract/tessdata/";
  std::regex rgx_pref("[0-9]\\.[0-9][0-9][0-9]");
  std::regex rgx_gen("[0-9]\\.?[0-9][0-9][0-9]|MEN|WOMEN|STAIR[1-2]?|GENDER");
  std::vector<std::string> class_names;
  class_names.push_back("Placard");

  YoloV5Observer observer(yolo_onnx, 1280, 102000, class_names, "output");
  PlacardDatabase placards(0.05, 0.9, trial_dir + "/placard_observations.csv");
  cv::dnn::Net east_net = cv::dnn::readNet(east_net_f);
  tesseract::TessBaseAPI* ocr = new tesseract::TessBaseAPI();
  ocr->Init(tess_dir.c_str(), "eng", tesseract::OEM_LSTM_ONLY);
  ocr->SetPageSegMode(tesseract::PSM_AUTO);
  ocr->SetVariable("tessedit_char_whitelist", "0123456789ADEIGMNORSTW\\.");
  ocr->SetVariable("debug_file", "tesseract.log");

  KinectPlayback playback(video);
  k4a::calibration k4a_calibration = playback.GetCalibration();

    KFRCaptureToTimeStamp kfrCaptureToTimeStamp;
    KFRCaptureToBGRA kfrCaptureToBGRA;
    FRBGRAToRGB frBGRAToRGB("BGRA4K", "RGB4K");
    KFRCaptureToDepth kfrCaptureToDepth(&k4a_calibration);
    KFRDepthToPointcloud kfrDepthToPointcloud(&k4a_calibration);
    FRRGB4KTo720p frRGB4KTo720p;
    FRGetBoundingBoxes frGetBoundingBoxes((Observer*)&observer, "RGB720p");
    FRExtractPlacard frExtractPlacard(placards, &east_net, ocr, 0.5, 0.4, rgx_pref, rgx_gen, "RGB720p");

    // kfrGetBoundingBoxes.setFrameRecipient((FrameRecipient*)&kfrExtractPlacard);
    frRGB4KTo720p.setFrameRecipient((FrameRecipient*)&frGetBoundingBoxes);
    kfrDepthToPointcloud.setFrameRecipient((FrameRecipient*)&frRGB4KTo720p);
    kfrCaptureToDepth.setFrameRecipient((FrameRecipient*)&kfrDepthToPointcloud);
    frBGRAToRGB.setFrameRecipient((FrameRecipient*)&kfrCaptureToDepth);
    kfrCaptureToBGRA.setFrameRecipient((FrameRecipient*)&frBGRAToRGB);
    kfrCaptureToTimeStamp.setFrameRecipient((FrameRecipient*)&kfrCaptureToBGRA);
    playback.setFrameRecipient((FrameRecipient*)&kfrCaptureToTimeStamp);

  playback.start();
  playback.getDataPacket().rgb_to_world = Eigen::Affine3f(Eigen::Matrix4f::Identity(4,4));

  std::string input = "";
  while(input != "c") {
    std::cout << "Enter the time stamps for viewing seperated by a comma (no space), enter 'c' to cancel.\n";
    std::cin >> input;
    std::vector<std::chrono::microseconds> times;
    parse_line(input, times);
    for (auto time : times) {
      playback.seekFrame(time);
      playback.doOnce();
      cv::Mat img = playback.getDataPacket()._mats["RGB720p"];
      auto bboxes = playback.getDataPacket()._boxes;
      for (BoundingBox bbox : bboxes) {
        observer.draw_label(img, bbox);
      }
      cv::imshow(std::to_string(time.count()), img);
    }
    cv::waitKey(0);
    cv::destroyAllWindows();
  }

  return 0;
}