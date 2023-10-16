#include <opencv2/opencv.hpp>
#include <opencv2/dnn/dnn.hpp>
#include <fstream>

#include "yolov5_observer.h"
#include "bounding_box.h"

using namespace cv;
using namespace std;
using namespace cv::dnn;

int main() {
  Mat frame;
  frame = imread("data/test/0000000962.jpg");
  std::string onnxfile = "config/yolov6s.onnx";
  std::string label_file = "config/coco-labels.txt";
  YoloV5Observer observer(onnxfile, 640, 3072, label_file, "outputs");
  std::vector<BoundingBox> bboxes;
  observer.getDetections(frame, bboxes);
  std::cout << "Identified " << bboxes.size() << " objects in frame\n";
  Mat img = frame.clone();
  for (BoundingBox box : bboxes) {
    observer.draw_label(img, box);
  }
  
  namedWindow("Output", cv::WINDOW_NORMAL);
  imshow("Output", img);
  cv::resizeWindow("Output", 1280, 720);
  waitKey(0);
  return 0;
}