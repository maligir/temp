#include "fr_record_frames.h"
#include "data_packet.h"

// #include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>

FRRecordFrames::FRRecordFrames(const std::string& rgb,
                                const std::string& depth,
                                const std::string& save_dir) :
    _rgb(rgb)
    , _depth(depth)
    , _save_dir(save_dir)
    , _count(0) {}

FRRecordFrames::~FRRecordFrames() {
  _writer.close();
}

void FRRecordFrames::initializePacket(DataPacket *dp) {
  boost::filesystem::create_directories(_save_dir+"/"+_rgb);
  boost::filesystem::create_directories(_save_dir+"/"+_depth);

  _writer.open(_save_dir+"img_index.txt");
  _writer << "Timestamp," << _rgb << "," << _depth << std::endl;

  if (_fr) {
    _fr->initializePacket(dp);
  }
}

void FRRecordFrames::receiveFrame(DataPacket *dp) {
  _count++;
  cv::Mat img1 = dp->_mats[_rgb];
  cv::Mat img2 = dp->_mats[_depth];
  const size_t timestamp = dp->tframe.count();

  std::string save1 = _save_dir+"/"+_rgb+"/"+std::to_string(_count)+".tiff";
  std::string save2 = _save_dir+"/"+_depth+"/"+std::to_string(_count)+".tiff";

  // cv::Mat img2_reinterp = cv::Mat(img2.rows, img2.cols, CV_8UC4, img2.data);

  cv::imwrite(save1, img1);
  cv::imwrite(save2, img2);

  _writer << std::to_string(timestamp) << "," << save1 << "," << save2 << std::endl;

  if (_fr) {
    _fr->receiveFrame(dp);
  }
}