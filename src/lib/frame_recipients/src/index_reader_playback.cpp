#include "index_reader_playback.h"

#include <unistd.h>
#include <stdio.h>
#include <limits.h>

IndexReaderPlayback::IndexReaderPlayback(const std::string& index): _index(index) {}

IndexReaderPlayback::~IndexReaderPlayback() {
  _reader.close();
}

void IndexReaderPlayback::start() {
  _reader.open(_index+"img_index.txt");
  if (!_reader.is_open() || _reader.bad()) {
    std::cout << "unable to open file " << _index+"img_index.txt\n";
    exit(1);
  }
  std::string header;
  std::getline(_reader,header);

  if (header == "") {
    std::cout << "unable to read header line of file " << _index+"/img_index.txt\n";
    exit(1);
  }

  std::string delim = ",";
  // erase timestamp header
  header.erase(0, header.find(delim) + delim.length());


  _rgb = header.substr(0, header.find(delim));
  header.erase(0, header.find(delim) + delim.length());

  _depth = header.substr(0, header.find(delim));

  if (_fr) {
    _fr->initializePacket(&_pkt);
  }
}

bool IndexReaderPlayback::isFinished() {
  return !_reader.good();
}

void IndexReaderPlayback::seekFrame(const std::chrono::microseconds& tstamp) {
  // if the start time occurs before the current time, restart the stream
  if (tstamp < _pkt.tframe) {
    _reader.close();
    start();
  }

  // recorded times from the last two lines
  std::chrono::microseconds Tnm1(0);
  std::chrono::microseconds Tnm2(0);

  // stream pointer to start of last two lines
  std::streampos ptr_nm1 = _reader.tellg();
  std::streampos ptr_nm2 = _reader.tellg();

  // find line such that Tnm2 < tstamp <= Tnm1
  std::string line;
  while (Tnm1 < tstamp) {
    std::getline(_reader, line);
    std::string t_str = line.substr(0,line.find(","));
    std::chrono::microseconds Tn(std::stoul(t_str));
    Tnm2 = Tnm1;
    Tnm1 = Tn;

    ptr_nm2 = ptr_nm1;
    ptr_nm1 = _reader.tellg();
  }

  std::chrono::microseconds d1 = Tnm1 - tstamp;
  std::chrono::microseconds d2 = tstamp - Tnm2;

  if (d1 <= d2) {
    _reader.seekg(ptr_nm1);
  } else {
    _reader.seekg(ptr_nm2);
  }

}

DataPacket& IndexReaderPlayback::getDataPacket() {
  return _pkt;
}

void IndexReaderPlayback::doOnce() {
  std::string line;
  std::getline(_reader, line);

  if (line == "") {
    std::cerr << "WARNING: index file contains empty line\n";
    return;
  }

  std::string delim = ",";

  std::string tstamp = line.substr(0, line.find(delim));
  line.erase(0, line.find(delim) + delim.length());

  std::string frgb = line.substr(0, line.find(delim));
  line.erase(0, line.find(delim) + delim.length());

  std::string fdepth = line.substr(0, line.find(delim));

  _pkt.tframe = std::chrono::microseconds(std::stoul(tstamp));
  _pkt._mats[_rgb] = cv::imread(frgb, cv::IMREAD_COLOR);
  _pkt._mats[_depth] = cv::imread(fdepth, cv::IMREAD_ANYDEPTH);

  if (_fr) {
    _fr->receiveFrame(&_pkt);
  }
}
