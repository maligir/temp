#include "yolo_observer.h"

#include <iostream>

YoloObserver::YoloObserver(const std::string& cfgfile,
                           const std::string& weightfile):
    nClasses_(1)
    , demoFrame_(3)
    , hier_(0.5) {
  char* weights = new char[weightfile.length() + 1];
  strcpy(weights, weightfile.c_str());

  cfg = new char[cfgfile.length() + 1];
  strcpy(cfg, cfgfile.c_str());

  std::cout << "loading network\n" << "config: " << cfgfile << "\nweights: " << weightfile << std::endl;

  net_ = load_network(cfg, weights, 0);
  set_batch_network(net_, 1);

  demoTotal_ = sizeNetwork(net_);
  predictions_ = (float**)calloc(demoFrame_, sizeof(float*));
  for (int i = 0; i < demoFrame_; ++i) {
    predictions_[i] = (float*)calloc(demoTotal_, sizeof(float));
  }
  avg_ = (float*)calloc(demoTotal_, sizeof(float));
}

void YoloObserver::getDetections(cv::Mat& img,
                                 std::vector<BoundingBox>& bboxes) {
  // IplImage ipl_in = img;
  image buff = mat_to_image(&img);
  image buffLetter = letterbox_image(buff, net_->w, net_->h);
  layer l = net_->layers[net_->n - 1];
  network_predict(net_, buffLetter.data);
  int nboxes = 0;
  detection *dets = get_network_boxes(net_, buff.w, buff.h, 0.05, 0.5, 0, 1, &nboxes);

  // extract the bounding boxes
  for (int i = 0; i < nboxes; ++i) {
    float xmin = dets[i].bbox.x - dets[i].bbox.w / 2.;
    float xmax = dets[i].bbox.x + dets[i].bbox.w / 2.;
    float ymin = dets[i].bbox.y - dets[i].bbox.h / 2.;
    float ymax = dets[i].bbox.y + dets[i].bbox.h / 2.;

    if (xmin < 0) xmin = 0;
    if (ymin < 0) ymin = 0;
    if (xmax > 1) xmax = 1;
    if (ymax > 1) ymax = 1;

    float BoundingBox_width = xmax - xmin;
    float BoundingBox_height = ymax - ymin;

    // iterate through possible boxes and collect the bounding boxes
    for (int j = 0; j < nClasses_; ++j) {
      if (dets[i].prob[j]) {
        if (BoundingBox_width > 0.01 && BoundingBox_height > 0.01) {
          BoundingBox bbox(dets[i].prob[j], xmin, ymin, xmax, ymax, j, "placard");
          bboxes.push_back(bbox);
        }
      }
    }
  }

  free(buff.data);
  free(buffLetter.data);
}

int YoloObserver::sizeNetwork(network* net) {
  int i;
  int count = 0;
  for (i = 0; i < net->n; ++i) {
    layer l = net->layers[i];
    if (l.type == YOLO || l.type == REGION || l.type == DETECTION) {
      count += l.outputs;
    }
  }
  return count;
}