#ifndef KFR_EXTRACT_PLACARD_H
#define KFR_EXTRACT_PLACARD_H

#include "frame_recipient.h"
#include "placard_database.h"
#include <string>
#include <opencv2/highgui.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/dnn.hpp>
#include <tesseract/baseapi.h>

#include <regex>

#include "bounding_box.h"

class FRExtractPlacard : public FrameRecipient {
public:
    FRExtractPlacard(PlacardDatabase& data,
                     cv::dnn::Net* east_net,
                     tesseract::TessBaseAPI* ocr,
                     float conf_thr,
                     float nms_thr,
                     std::regex& rgx_pref,
                     std::regex& rgx_gen,
                     const std::string& img_name);

    void initializePacket(DataPacket *dp);
    void receiveFrame(DataPacket *dp);

protected:
    PlacardDatabase& _data;
    cv::dnn::Net* _east_net;
    tesseract::TessBaseAPI* _ocr;
    const float _conf_thr;
    const float _nms_thr;
    const std::regex& _regex_pref;
    const std::regex& _regex_gen;
    const std::string _img_name;

    std::string extractText(cv::Mat& warped);

    void normalToQuaternion(const Eigen::Vector3f& normal,
                            Eigen::Quaternionf& quat);
    bool getPlacardPose(const int16_t* buffer,
                        int w, int h,
                        BoundingBox& bb,
                        Eigen::Vector3f& centroid,
                        Eigen::Vector3f& normal);
    void decode(const cv::Mat& scores,
            const cv::Mat& geometry,
            float scoreThresh,
            std::vector<cv::RotatedRect>& detections,
            std::vector<float>& confidences);
    void applyEastNet(cv::Mat patch, std::vector<cv::Rect>& text_patches);
    std::string applyTesseract(cv::Mat patch);
    void warpImage(cv::Mat& img_raw,
                   Eigen::Vector3f centroid,
                   Eigen::Vector3f normal,
                   cv::Mat& warped,
                   cv::Rect& roi);
    void padPatch(cv::Mat& patch, cv::Mat& padded);
    void bb2roi(BoundingBox& bb, cv::Rect& roi);
};

#endif