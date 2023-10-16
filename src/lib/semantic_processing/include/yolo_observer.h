#ifndef YOLO_OBSERVER_H
#define YOLO_OBSERVER_H

#include <string>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include "bounding_box.h"

#include "observer.h"

#ifdef GPU
#include "cublas_v2.h"
#include "cuda_runtime.h"
#include "curand.h"
#endif

extern "C" {
#include <sys/time.h>
#include "box.h"
#include "cost_layer.h"
#include "image.h"
#include "detection_layer.h"
#include "network.h"
#include "parser.h"
#include "region_layer.h"
#include "utils.h"
}

extern "C" image mat_to_image(cv::Mat* src);
// extern "C" void show_image_cv(image p, const char* name, IplImage* disp);

class YoloObserver : public Observer {
public:
  YoloObserver(const std::string& cfgfile,
               const std::string& weightfile);

  void getDetections(cv::Mat& image,
                     std::vector<BoundingBox>& bboxes);

private:
  int sizeNetwork(network* net);

  char* weights;
  char* cfg;
  network* net_;
  const unsigned int nClasses_;
  int demoTotal_;
  int demoFrame_;
  float** predictions_;
  float* avg_;
  const float hier_;
};

#endif