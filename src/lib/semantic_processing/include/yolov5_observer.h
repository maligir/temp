#ifndef YOLOV5_OBSERVER_H
#define YOLOV5_OBSERVER_H

#include "observer.h"
#include "bounding_box.h"
#include <opencv2/opencv.hpp>
#include <opencv2/dnn/dnn.hpp>
#include <fstream>

class YoloV5Observer : public Observer {
public:
  YoloV5Observer(const std::string& onnxfile,
                 const float size,
                 const unsigned int nrows,
                 const std::vector<std::string>& class_names,
                 const std::string& outLayer);

  YoloV5Observer(const std::string& onnxfile,
                 const float size,
                 const unsigned int nrows,
                 const std::string& flabels,
                 const std::string& outLayer);

  void getDetections(cv::Mat& image,
                     std::vector<BoundingBox>& bboxes);
  
  void dumpNetwork();

  void draw_label(cv::Mat& input_image,
                  BoundingBox& bbox);
private:

  cv::Mat pre_process(cv::Mat &input_image);
  void post_process(cv::Mat &input_image,
                    cv::Mat &outputs,
                    std::vector<BoundingBox>& bboxes);
  unsigned int read_labels(const std::string& flabels);
  
  std::vector<std::string> _class_names;
  const float _size;
  const unsigned int _nrows;
  const unsigned int _nclasses;

  cv::dnn::Net _net;
  const std::string _outLayer;
};

#endif