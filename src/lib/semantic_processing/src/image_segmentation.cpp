#include "image_segmentation.h"

using namespace caffe;
using std::string;

SegmentationObserver::SegmentationObserver(
    const string& model_file,
    const string& trained_file) {
  #ifdef CPU_ONLY
  Caffe::set_mode(Caffe::CPU);
  #else
  Caffe::set_mode(Caffe::GPU);
  #endif
  
  std::cout << "Segmentation Network File Location:\n" << model_file << std::endl;
  net_.reset(new Net<float>(model_file, TEST));
  net_->CopyTrainedLayersFrom(trained_file);
  
  CHECK_EQ(net_->num_inputs(), 1)
      << "Network should have exactly one input.";
  CHECK_EQ(net_->num_outputs(), 1)
      << "Network should have exactly one output.";
  
  Blob<float>* input_layer = net_->input_blobs()[0];
  num_channels_ = input_layer->channels();
  CHECK(num_channels_ == 3 || num_channels_ == 1)
      << "Input layer should have 1 or 3 channels.";
  input_geometry_ = cv::Size(input_layer->width(),
                             input_layer->height());
}

void SegmentationObserver::SegmentImage(const cv::Mat& img,
                                        cv::Mat* out) {
  Blob<float>* input_layer = net_->input_blobs()[0];
  input_layer->Reshape(1, num_channels_,
                       input_geometry_.height, input_geometry_.width);
  /* Forward dimension change to all layers. */
  net_->Reshape();
  
  std::vector<cv::Mat> input_channels;
  WrapInputLayer(&input_channels);
  
  Preprocess(img, &input_channels);
  
  net_->Forward();
  Blob<float>* output_layer = net_->output_blobs()[0];
  
  const float* begin = output_layer->cpu_data();
  float* farr = const_cast<float*>(begin);
  
  int nfloats = output_layer->count();
  
  int width = input_layer->width();
  int height = input_layer->height();

  CHECK(nfloats == width*height)
      << "Output size should match number of pixels";
  
  cv::Mat output(height, width, CV_32FC1, farr);
  // convert 32F to 8U
  double min, max;
  cv::minMaxLoc(output, &min, &max);
  if (min!=max){ 
    output -= min;
    output.convertTo(*out, CV_8U, 255.0 / (max - min));
  } else{
    std::cout << "Output min == max, output all zero" << std::endl;
    output.convertTo(*out, CV_8U); // going to be 0 but correct dim
  }
}

void SegmentationObserver::WrapInputLayer(
    std::vector<cv::Mat>* input_channels) {
  Blob<float>* input_layer = net_->input_blobs()[0];
  
  int width = input_layer->width();
  int height = input_layer->height();
  float* input_data = input_layer->mutable_cpu_data();
  for (int i = 0; i < input_layer->channels(); ++i) {
    cv::Mat channel(height, width, CV_32FC1, input_data);
    input_channels->push_back(channel);
    input_data += width * height;
  }
}

void SegmentationObserver::Preprocess(
    const cv::Mat& img,
    std::vector<cv::Mat>* input_channels) {
  /* Convert the input image to the input image format of the network. */
  cv::Mat sample;
  if (img.channels() == 3 && num_channels_ == 1)
    cv::cvtColor(img, sample, cv::COLOR_BGR2GRAY);
  else if (img.channels() == 4 && num_channels_ == 1)
    cv::cvtColor(img, sample, cv::COLOR_BGRA2GRAY);
  else if (img.channels() == 4 && num_channels_ == 3)
    cv::cvtColor(img, sample, cv::COLOR_BGRA2BGR);
  else if (img.channels() == 1 && num_channels_ == 3)
    cv::cvtColor(img, sample, cv::COLOR_GRAY2BGR);
  else
    sample = img;
  
  cv::Mat sample_resized;
  if (sample.size() != input_geometry_)
    cv::resize(sample, sample_resized, input_geometry_);
  else
    sample_resized = sample;
  
  cv::Mat sample_float;
  if (num_channels_ == 3)
    sample_resized.convertTo(sample_float, CV_32FC3);
  else
    sample_resized.convertTo(sample_float, CV_32FC1);
  
  /* This operation will write the separate BGR planes directly to the
   * input layer of the network because it is wrapped by the cv::Mat
   * objects in input_channels. */
  cv::split(sample_float, *input_channels);
  
  CHECK(reinterpret_cast<float*>(input_channels->at(0).data)
  == net_->input_blobs()[0]->cpu_data())
      << "Input channels are not wrapping"
      << " the input layer of the network.";
}

