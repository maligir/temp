#ifndef OBSERVER_H
#define OBSERVER_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "bounding_box.h"

class Observer {
public:
  virtual void getDetections(cv::Mat& image, std::vector<BoundingBox>& bboxes) = 0;
};

#endif