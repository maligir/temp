#include "yolov5_observer.h"

using namespace cv;
using namespace std;
using namespace cv::dnn;

// Constants.
const float SCORE_THRESHOLD = 0.5;
const float NMS_THRESHOLD = 0.45;
const float CONFIDENCE_THRESHOLD = 0.05;

// Text parameters.
const float FONT_SCALE = 0.7;
const int FONT_FACE = FONT_HERSHEY_SIMPLEX;
const int THICKNESS = 1;

// Colors.
Scalar BLACK = Scalar(0,0,0);
Scalar BLUE = Scalar(255, 178, 50);
Scalar YELLOW = Scalar(0, 255, 255);
Scalar RED = Scalar(0,0,255);

YoloV5Observer::YoloV5Observer(const std::string& onnxfile,
                               const float size,
                               const unsigned int nrows,
                               const vector<string>& class_names,
                               const std::string& outLayer):
    _size(size)
    , _nrows(nrows)
    , _nclasses(class_names.size())
    , _class_names(class_names)
    , _outLayer(outLayer) {
  _net = readNetFromONNX(onnxfile);
}

YoloV5Observer::YoloV5Observer(const std::string& onnxfile,
                               const float size,
                               const unsigned int nrows,
                               const string& flabels,
                               const std::string& outLayer):
    _size(size)
    , _nrows(nrows)
    , _nclasses(read_labels(flabels))
    , _outLayer(outLayer) {
  _net = readNetFromONNX(onnxfile);
}

void YoloV5Observer::dumpNetwork() {
  std::cout << _net.dump() << std::endl;
}

unsigned int YoloV5Observer::read_labels(const string& flabels) {
  ifstream infile("thefile.txt");
  std::string line;
  _class_names.clear();
  while (std::getline(infile, line)) {
    _class_names.push_back(line);
  }
  infile.close();
  return _class_names.size();
}

void YoloV5Observer::draw_label(Mat& input_image,
                                BoundingBox& bbox) {
// Display the label at the top of the bounding box.
  int baseLine;
  Size label_size = getTextSize(bbox.Class, FONT_FACE, FONT_SCALE, THICKNESS, &baseLine);
  int top = input_image.rows*bbox.ymin;
  int left = input_image.cols*bbox.xmin;
  int bottom = input_image.rows*bbox.ymax;
  int right = input_image.cols*bbox.xmax;
  // Top left corner.
  Point tlc = Point(left, top - label_size.height - baseLine);
  // Bottom right corner.
  Point brc = Point(left + label_size.width, top - baseLine);
  // draw blue bounding box
  rectangle(input_image, Point(left, top), Point(right, bottom), BLUE, 3*THICKNESS);
  // Draw white rectangle.
  rectangle(input_image, tlc, brc, BLACK, FILLED);
  // Put the label on the black rectangle.
  putText(input_image, bbox.Class, Point(left, top - baseLine), FONT_FACE, FONT_SCALE, YELLOW, THICKNESS);
}

Mat YoloV5Observer::pre_process(Mat &input_image) {
  Mat blob = blobFromImage(input_image, 1./255., Size(_size, _size), Scalar(), true, false);

  _net.setInput(blob);

  return _net.forward(_outLayer);
}

void YoloV5Observer::post_process(Mat &input_image,
                                  Mat &output,
                                  vector<BoundingBox>& bboxes) {
  vector<int> class_ids;
  vector<float> confidences;
  vector<Rect> boxes;
  // Resizing factor.
  float *data = (float *)output.data;
  const int dimensions = 5 + _nclasses;
  float x_factor = input_image.cols / _size;
  float y_factor = input_image.rows / _size;
  for (int i = 0; i < _nrows; ++i) {
    float confidence = data[4];
    if (confidence >= CONFIDENCE_THRESHOLD) {
      float * classes_scores = data + 5;
      // Create a 1x(5+nclasses) Mat and store class scores of nclasses classes.
      Mat scores(1, 5+_class_names.size(), CV_32FC1, classes_scores);
      Point class_id;
      double max_class_score;
      minMaxLoc(scores, 0, &max_class_score, 0, &class_id);

      if (max_class_score > SCORE_THRESHOLD) {

        confidences.push_back(confidence);
        class_ids.push_back(class_id.x);

        float cx = data[0];
        float cy = data[1];
        float w = data[2];
        float h = data[3];

        int left = int((cx - 0.5 * w) * x_factor);
        int top = int((cy - 0.5 * h) * y_factor);
        int width = int(w * x_factor);
        int height = int(h * y_factor);

        boxes.push_back(Rect(left, top, width, height));
      }
    }
    // Jump to the next row.
    data += dimensions;
  }

  vector<int> indices;
  NMSBoxes(boxes, confidences, SCORE_THRESHOLD, NMS_THRESHOLD, indices);
  for (int i = 0; i < indices.size(); i++) {
    int idx = indices[i];
    Rect& box = boxes[idx];
    float conf = confidences[idx];

    float w = static_cast<float>(box.width);
    float h = static_cast<float>(box.height);

    float l = (static_cast<float>(box.x))/input_image.cols;
    float r = (static_cast<float>(box.x) + w)/input_image.cols;
    float t = (static_cast<float>(box.y))/input_image.rows;
    float b = (static_cast<float>(box.y) + h)/input_image.rows;

    BoundingBox bbox(conf,
                     l, t, r, b,
                     class_ids[idx],
                     "Placard");
    bboxes.push_back(bbox);
  }
}

void YoloV5Observer::getDetections(cv::Mat& image,
                                   std::vector<BoundingBox>& bboxes) {
  Mat output = pre_process(image);
  post_process(image, output, bboxes);
}