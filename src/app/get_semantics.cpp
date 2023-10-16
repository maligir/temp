#include "kinect_playback.h"
#include "rosbag_playback.h"
#include "kinect_frame_recipient.h"
#include "kfr_capture_to_timestamp.h"
#include "kfr_capture_to_bgra.h"
#include "kfr_capture_to_depth.h"
#include "rfr_get_frame_info.h"
#include "rfr_sync_image_frames.h"
#include "fr_bgra_to_rgb.h"
#include "fr_rgb_4k_to_720p.h"
#include "kfr_depth_to_pointcloud.h"
#include "fr_get_bounding_boxes.h"
#include "fr_extract_placard.h"
#include "fr_depth_to_pointcloud.h"
#include "octmap_utils.h"
#include "orbslam_utils.h"
#include "yolov5_observer.h"
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

#include <fstream>
#include <string>

void ParseKeyframe(std::string& line,
                   std::chrono::microseconds& ms,
                   Eigen::Vector3f& pose,
                   Eigen::Quaternionf& quat) {
    size_t pos = 0;
    std::string token;
    std::string delimiter = ",";

    // frame ID
    pos = line.find(delimiter);
    token = line.substr(0, pos);
    line.erase(0, pos + delimiter.length());

    // timestamp
    pos = line.find(delimiter);
    token = line.substr(0, pos);
    line.erase(0, pos + delimiter.length());
    float ts = std::stof(token);
    ms = std::chrono::microseconds(static_cast<size_t>(1e6*ts));

    // x pos
    pos = line.find(delimiter);
    token = line.substr(0, pos);
    line.erase(0, pos + delimiter.length());
    pose.x() = std::stof(token);

    // y pos
    pos = line.find(delimiter);
    token = line.substr(0, pos);
    line.erase(0, pos + delimiter.length());
    pose.y() = std::stof(token);

    // z pos
    pos = line.find(delimiter);
    token = line.substr(0, pos);
    line.erase(0, pos + delimiter.length());
    pose.z() = std::stof(token);

    // x quat
    pos = line.find(delimiter);
    token = line.substr(0, pos);
    line.erase(0, pos + delimiter.length());
    quat.x() = std::stof(token);

    // y quat
    pos = line.find(delimiter);
    token = line.substr(0, pos);
    line.erase(0, pos + delimiter.length());
    quat.y() = std::stof(token);

    // z quat
    pos = line.find(delimiter);
    token = line.substr(0, pos);
    line.erase(0, pos + delimiter.length());
    quat.z() = std::stof(token);

    // w quat
    pos = line.find(delimiter);
    token = line.substr(0, pos);
    line.erase(0, pos + delimiter.length());
    quat.w() = std::stof(token);
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

    std::string img_key;
    std::string depth_key;
    std::string trial_dir;
    settings.getRequiredParameter("Save.Path", trial_dir);

    if (sensor == "K4A") {
        std::string k4a_rec;
        settings.getRequiredParameter("Record.Path", k4a_rec);
    
        KinectPlayback* kinect = new KinectPlayback(k4a_rec);
        k4a::calibration* k4a_calibration = new k4a::calibration(kinect->GetCalibration());

        KFRCaptureToTimeStamp* kfrCaptureToTimeStamp = new KFRCaptureToTimeStamp();
        KFRCaptureToBGRA* kfrCaptureToBGRA = new KFRCaptureToBGRA();
        FRBGRAToRGB* frBGRAToRGB = new FRBGRAToRGB("BGRA4K", "RGB4K");
        KFRCaptureToDepth* kfrCaptureToDepth = new KFRCaptureToDepth(k4a_calibration);
        FRRGB4KTo720p* frRGB4KTo720p = new FRRGB4KTo720p();
        KFRDepthToPointcloud* kfrDepthToPointcloud = new KFRDepthToPointcloud(k4a_calibration);
    
        frRGB4KTo720p->setFrameRecipient((FrameRecipient*)kfrDepthToPointcloud);
        kfrCaptureToDepth->setFrameRecipient((FrameRecipient*)frRGB4KTo720p);
        frBGRAToRGB->setFrameRecipient((FrameRecipient*)kfrCaptureToDepth);
        kfrCaptureToBGRA->setFrameRecipient((FrameRecipient*)frBGRAToRGB);
        kfrCaptureToTimeStamp->setFrameRecipient((FrameRecipient*)kfrCaptureToBGRA);
        kinect->setFrameRecipient((FrameRecipient*)kfrCaptureToTimeStamp);

        playback = kinect;

        img_key = "RGB720p";
    } else if (sensor == "ROS") {
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

        img_key = "Image";
        depth_key = "Depth";

        std::vector<std::string> topics({img_info_topic,
                                        img_topic,
                                        depth_info_topic,
                                        depth_topic});
        ROSPlayback* ros = new ROSPlayback(rosbag_name, topics);
        RFRGetFrameInfo* get_img_info = new RFRGetFrameInfo(img_info_topic, img_key);
        RFRGetFrameInfo* get_depth_info = new RFRGetFrameInfo(depth_info_topic, depth_key);
        ImageRes img_res(img_height, img_width, 3, 1);
        ImageRes depth_res(depth_height, depth_width, 1, 4);
        RFRSyncImageFrames* image_sync = new RFRSyncImageFrames(
            img_topic,
            depth_topic,
            img_key,
            depth_key,
            img_res,
            depth_res,
            CV_8UC3,
            CV_32FC1);
        FRDepthToPointcloud* depth_to_pc = new FRDepthToPointcloud(depth_key);
        
        image_sync->setFrameRecipient((FrameRecipient*)depth_to_pc);
        get_depth_info->setFrameRecipient((FrameRecipient*)image_sync);
        get_img_info->setFrameRecipient((FrameRecipient*)get_depth_info);
        ros->setFrameRecipient((FrameRecipient*)get_img_info);

        playback = ros;
    } else {
        std::cerr << "No implementation available for sensor type " << sensor << std::endl;
        return 1;
    }

    settings.readTextRecognitionSettings();
    std::string keyframes_p;
    // std::string yolo_cfg;
    std::string yolo_wts;
    std::string east_net_f;
    std::string tess_dir;
    std::string tess_white;
    std::string rgx1;
    std::string rgx2;
    std::string yolo_out;
    int yolo_size;
    int yolo_rows;
    float thr_conf;
    float thr_nms;

    // settings.getRequiredParameter("YOLO.Config", yolo_cfg);
    settings.getRequiredParameter("YOLO.Weights", yolo_wts);
    settings.getRequiredParameter("YOLO.Output", yolo_out);
    settings.getRequiredParameter("YOLO.Size", yolo_size);
    settings.getRequiredParameter("YOLO.Rows", yolo_rows);
    settings.getRequiredParameter("EAST.Net", east_net_f);
    settings.getRequiredParameter("Tess.Data", tess_dir);
    settings.getRequiredParameter("Tess.Whitelist", tess_white);
    settings.getRequiredParameter("Keyframes.Path", keyframes_p);
    settings.getRequiredParameter("Regex1", rgx1);
    settings.getRequiredParameter("Regex2", rgx2);
    settings.getRequiredParameter("Threshold.Confidence", thr_conf);
    settings.getRequiredParameter("Threshold.NMS", thr_nms);

    // placard reading regex
    std::regex rgx_pref(rgx1);
    std::regex rgx_gen(rgx2);

    Eigen::Matrix4f axis_flip = Eigen::MatrixXf::Zero(4,4);
    axis_flip(0,0) = 1;
    axis_flip(1,2) = 1;
    axis_flip(2,1) = -1;
    axis_flip(3,3) = 1;

    std::vector<std::string> class_names;
    class_names.push_back("Placard");
    YoloV5Observer observer(yolo_wts, yolo_size, yolo_rows, class_names, yolo_out);
    PlacardDatabase placards(0.05, 0.9, trial_dir + "/placard_observations.csv");
    cv::dnn::Net east_net = cv::dnn::readNet(east_net_f);
    tesseract::TessBaseAPI* ocr = new tesseract::TessBaseAPI();
    ocr->Init(tess_dir.c_str(), "eng", tesseract::OEM_LSTM_ONLY);
    ocr->SetPageSegMode(tesseract::PSM_AUTO);
    ocr->SetVariable("tessedit_char_whitelist", tess_white.c_str());
    ocr->SetVariable("debug_file", "tesseract.log");

    FRGetBoundingBoxes frGetBoundingBoxes((Observer*)&observer, img_key);
    FRExtractPlacard frExtractPlacard(placards, &east_net, ocr, thr_conf, thr_nms, rgx_pref, rgx_gen, img_key);

    frGetBoundingBoxes.setFrameRecipient((FrameRecipient*)&frExtractPlacard);
    playback->getLastFR()->setFrameRecipient((FrameRecipient*)&frGetBoundingBoxes);

    playback->start();

    std::ifstream keyframes_f(keyframes_p);
    std::string line;
    std::string ofile = trial_dir + "/octomap.bt";
    std::string fmap = trial_dir + "/occupation.csv";
    std::string fplacs = trial_dir + "/avg_placards.csv";
    std::string fplacs_all = trial_dir + "/all_placards.csv";
    octomap::OcTree octree(0.03);
    size_t count = 0;
    while (std::getline(keyframes_f, line)) {
        std::chrono::microseconds tstamp;
        Eigen::Vector3f pose;
        Eigen::Quaternionf quat;
        ParseKeyframe(line, tstamp, pose, quat);

        count++;
        Eigen::Affine3f kf_pose(quat);
        kf_pose.translation() = pose;
        playback->getDataPacket().rgb_to_world = Eigen::Affine3f(axis_flip)*kf_pose;
        // playback.getDataPacket().rgb_to_world = kf_pose;
        playback->seekFrame(tstamp);
        playback->doOnce();
        if (count % 100 == 0) {
            std::cout << "processed: " << count << " lines\n";
        }

        cv::Mat points;
        GetPointCloudInMapFrame(kf_pose.matrix(),
                                &(playback->getDataPacket()),
                                250,
                                2880,
                                points);
        octomap::Pointcloud pc;
        octomap::point3d so(pose(0),
                            pose(2),
                            0);
        CVMat2OcTreePC(points, -pose(1), pc);
        octree.insertPointCloud(pc, so, -1, true, true);
    }
    octree.updateInnerOccupancy();
    octree.writeBinary(ofile);
    OccupationMap2D occupation(2000,2000);
    OcTree2OccupationMap(octree, occupation);
    occupation.write(fmap);
    placards.write(fplacs);
    placards.write_all(fplacs_all);

    std::cout << "finished\n";
}