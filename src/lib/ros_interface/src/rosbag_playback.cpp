#include "rosbag_playback.h"

ROSPlayback::ROSPlayback(
  const std::string& fname,
  const std::vector<std::string>& topics) :
    _fname(fname)
    , _isFinished(false)
    , _topics(topics.begin(), topics.end()) {
  
  _bag.open(_fname, rosbag::bagmode::Read);
  _view.addQuery(_bag, rosbag::TopicQuery(_topics));
}

ROSPlayback::~ROSPlayback() {
  _bag.close();
}

void ROSPlayback::start() {
  _pkt._msg = _view.begin();
  if (_fr) {
    _fr->initializePacket((DataPacket*)&_pkt);
  }
}

bool ROSPlayback::isFinished() {
  return _isFinished;
}

void ROSPlayback::seekFrame(const std::chrono::microseconds& tstamp) {
  double target = 1e-6 * tstamp.count();
  for (auto m = _view.begin(); m != _view.end(); m++) {
    ros::Time t = m->getTime();
    double current = t.sec + 1e-9 * t.nsec;
    if (current >= target) {
      _isFinished = false;
      _pkt._msg = m;
      return;
    }
  }
  printf("No messages found at target time %f", target);
  _isFinished = true;
}

void ROSPlayback::doOnce() {
  if (!_isFinished) {
    // std::cout << "processing message on topic: " << _pkt._msg->getTopic() << std::endl;
    if (_fr) {
      _fr->receiveFrame((DataPacket*)&_pkt);
    }
    _pkt._msg++;
    if (_pkt._msg == _view.end()) {
      _isFinished = true;
    }
  } else {
    printf("End of file reached");
  }
}