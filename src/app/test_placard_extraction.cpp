#include "kinect_playback.h"
#include "kinect_frame_recipient.h"
#include "kfr_capture_to_timestamp.h"
#include "kfr_capture_to_bgra.h"
#include "kfr_capture_to_depth.h"
#include "fr_bgra_to_rgb.h"
#include "fr_rgb_4k_to_720p.h"
#include "kfr_depth_to_pointcloud.h"
#include "fr_get_bounding_boxes.h"
#include "fr_extract_placard.h"
#include "yolo_observer.h"

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

int main(int argc, char**argv) {
    if (argc < 2) {
        std::cerr << "\nUsage: ./play_video path_to_k4a_recording\n";
        return 1;
    }

    std::string yolo_cfg = "/home/david/placard_discovery/config/custom-yolov2-tiny-voc.cfg";
    std::string yolo_wts = "/home/david/placard_discovery/config/custom-yolov2-tiny-voc_best.weights";
    std::string east_model_f = "/home/david/placard_discovery/config/frozen_east_text_detection.pb";

    std::regex rgx_pref("[0-9]\\.[0-9][0-9][0-9]");
    std::regex rgx_gen("[0-9]\\.?[0-9][0-9][0-9]|MEN|WOMEN|STAIR");

    Eigen::Matrix4f axis_flip = Eigen::MatrixXf::Zero(4,4);
    axis_flip(0,0) = 1;
    axis_flip(1,2) = 1;
    axis_flip(2,1) = -1;
    axis_flip(3,3) = 1;

    YoloObserver observer(yolo_cfg, yolo_wts);
    PlacardDatabase placards(0.05, 0.9);
    tesseract::TessBaseAPI* ocr = new tesseract::TessBaseAPI();
    ocr->Init("/home/david/placard_discovery/third_party/tesseract/tessdata", "eng", tesseract::OEM_LSTM_ONLY);
    ocr->SetPageSegMode(tesseract::PSM_AUTO);
    ocr->SetVariable("tessedit_char_whitelist", "0123456789AEIMNORSTW\\.");
    ocr->SetVariable("debug_file", "tesseract.log");
    cv::dnn::Net east_net = cv::dnn::readNet(east_model_f);

    KinectPlayback playback(argv[1]);
    // clear view of placard
    // MEN sign
    playback.seekFrame(std::chrono::microseconds{344214577});
    // playback.seekFrame(std::chrono::microseconds{344281233});
    // playback.seekFrame(std::chrono::microseconds{345881233});

    // room 2.124 sign
    // playback.seekFrame(std::chrono::microseconds{510014576});
    // playback.seekFrame(std::chrono::microseconds{510747911});

    // debugging
    // playback.seekFrame(std::chrono::microseconds{1540000000});
    k4a::calibration k4a_calibration = playback.GetCalibration();

    KFRCaptureToTimeStamp kfrCaptureToTimeStamp;
    KFRCaptureToBGRA kfrCaptureToBGRA;
    FRBGRAToRGB frBGRAToRGB("BGRA4K", "RGB4K");
    FRRGB4KTo720p frRGB4KTo720p;
    KFRCaptureToDepth kfrCaptureToDepth(&k4a_calibration);
    KFRDepthToPointcloud kfrDepthToPointcloud(&k4a_calibration);
    FRGetBoundingBoxes frGetBoundingBoxes((Observer*)&observer, "RGB720p");
    FRExtractPlacard frExtractPlacard(placards, &east_net, ocr, 0.5, 0.4, rgx_pref, rgx_gen, "RGB720p");

    frGetBoundingBoxes.setFrameRecipient((FrameRecipient*)&frExtractPlacard);
    kfrDepthToPointcloud.setFrameRecipient((FrameRecipient*)&frGetBoundingBoxes);
    kfrCaptureToDepth.setFrameRecipient((FrameRecipient*)&kfrDepthToPointcloud);
    frRGB4KTo720p.setFrameRecipient((FrameRecipient*)&kfrCaptureToDepth);
    frBGRAToRGB.setFrameRecipient((FrameRecipient*)&frRGB4KTo720p);
    kfrCaptureToBGRA.setFrameRecipient((FrameRecipient*)&frBGRAToRGB);
    kfrCaptureToTimeStamp.setFrameRecipient((FrameRecipient*)&kfrCaptureToBGRA);
    playback.setFrameRecipient((FrameRecipient*)&kfrCaptureToTimeStamp);

    size_t nboxes = 0;
    while (true) {
        playback.getDataPacket().rgb_to_world = Eigen::Affine3f(axis_flip);
        playback.start();
        playback.doOnce();
        nboxes = playback.getDataPacket()._boxes.size();
        std::cout << "processed frame with timestamp: " << playback.getDataPacket().tframe.count() << std::endl;
        std::cout << "there are " << nboxes << " bounding boxes from yolo\n";

        cv::Mat img = playback.getDataPacket()._mats["RGB720p"];
        cv::Mat out;
        cv::cvtColor(img, out, cv::COLOR_RGB2BGR);
        cv::Mat& img_4K = playback.getDataPacket()._mats["BGRA4K"];
        for (BoundingBox bb : playback.getDataPacket()._boxes) {
            cv::Rect rect1(out.cols * bb.xmin,
                           out.rows * bb.ymin,
                           out.cols * (bb.xmax - bb.xmin),
                           out.rows * (bb.ymax - bb.ymin));
            cv::Rect rect2(img_4K.cols * bb.xmin,
                           img_4K.rows * bb.ymin,
                           img_4K.cols * (bb.xmax - bb.xmin),
                           img_4K.rows * (bb.ymax - bb.ymin));
            cv::rectangle(out, rect1, cv::Scalar(0, 0, 255));
            cv::Mat patch = img_4K(rect2);
            cv::imshow("placard bounding box", patch);
            cv::waitKey(0);
        }
        cv::imshow("frame", out);
        if (nboxes > 0) {
            cv::waitKey(0);
        } else {
            cv::waitKey(30);
        }
        break;
    }
    std::string f = "test_placard_write.txt";
    placards.write_all(f);
}