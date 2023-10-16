#include "rfr_sync_image_frames.h"

RFRSyncImageFrames::RFRSyncImageFrames(
    const std::string& img1_topic,
    const std::string& img2_topic,
    const std::string& cam1_name,
    const std::string& cam2_name,
    const ImageRes& res1,
    const ImageRes& res2,
    uint8_t cv_format1,
    uint8_t cv_format2):
    _res1(res1)
    , _res2(res2)
    , _img1_topic(img1_topic)
    , _img2_topic(img2_topic)
    , _cam1_name(cam1_name)
    , _cam2_name(cam2_name)
    , _cv_format1(cv_format1)
    , _cv_format2(cv_format2) {}

void RFRSyncImageFrames::initializePacket(ROSPacket *rp) {
  rp->_mats[_cam1_name] = cv::Mat(_res1.first, _res1.second, _cv_format1);
  rp->_mats[_cam2_name] = cv::Mat(_res2.first, _res2.second, _cv_format2);

  if (_fr) {
    _fr->initializePacket(rp);
  }
}

void RFRSyncImageFrames::receiveFrame(ROSPacket* rp) {
  sensor_msgs::ImageConstPtr img = NULL;

  if (rp->_msg->getTopic() == _img1_topic) {
    img = rp->_msg->instantiate<sensor_msgs::Image>();
    if (img != NULL) {
      // std::cout << "Found image on topic: " << _img1_topic << std::endl;
      _q1.push_back(img);
    }
  } else if (rp->_msg->getTopic() == _img2_topic) {
    img = rp->_msg->instantiate<sensor_msgs::Image>();
    if (img != NULL) {
      // std::cout << "Found image on topic: " << _img2_topic << std::endl;
      _q2.push_back(img);
    }
  }

  if (foundImageSync()) {
    // std::cout << "Images Sync'd\n";

    cv_bridge::CvImageConstPtr cv_ptr1;
    cv_bridge::CvImageConstPtr cv_ptr2;
    try {

      cv_ptr1 = cv_bridge::toCvShare(_q1[0], _q1[0]->encoding);
      cv_ptr2 = cv_bridge::toCvShare(_q2[0], _q2[0]->encoding);

      // std::cout << "depth image encoding: " << _q2[0]->encoding << std::endl;
      // std::cout << "depth image resolution: " << _q2[0]->height << "x" << _q2[0]->width << std::endl;

      // cv::imshow("Converted RGB Image", cv_ptr1->image);
      // cv::imshow("Converted Depth Image", cv_ptr2->image);

      memcpy(rp->_mats[_cam1_name].data, cv_ptr1->image.data, _res1.size());
      memcpy(rp->_mats[_cam2_name].data, cv_ptr2->image.data, _res2.size());

      // cv::imshow("Saved RGB Image", rp->_mats[_cam1_name]);
      // cv::imshow("Saved Depth Image", rp->_mats[_cam2_name]);
      // cv::waitKey(0);

      float t1 = _q1[0]->header.stamp.sec + 1e-9*_q1[0]->header.stamp.nsec;
      float t2 = _q2[0]->header.stamp.sec + 1e-9*_q2[0]->header.stamp.nsec;
      float t = (t1+t2)/2;
      rp->tframe = std::chrono::microseconds(static_cast<size_t>(1e6*t));

      _q1.pop_front();
      _q2.pop_front();

    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    if (_fr) {
      _fr->receiveFrame(rp);
    }
  }
}

bool RFRSyncImageFrames::foundImageSync() {
  size_t q1s = _q1.size();
  size_t q2s = _q2.size();
  if (q1s == 0 || q2s == 0) {
    return false;
  }
  double t1, t2;
  ros::Time rt1 = _q1[0]->header.stamp;
  ros::Time rt2 = _q2[0]->header.stamp;

  t1 = rt1.sec + 1e-9 * rt1.nsec;
  t2 = rt2.sec + 1e-9 * rt2.nsec;

  if (t1 > t2) {
    return searchQueueForTimeSync(t1, _q2);
  } else if (t1 < t2) {
    return searchQueueForTimeSync(t2, _q1);
  } else {
    return true;
  }
}

bool RFRSyncImageFrames::searchQueueForTimeSync(double time, ImageQueue& q) {
  if (q.size() < 2) {
    return false;
  }

  ros::Time rtend = q[q.size()-1]->header.stamp;
  double tend = rtend.sec + 1e-9 * rtend.nsec;
  if (tend < time) {
    q.erase(q.begin(), q.end()-1);
    return false;
  }

  while(q.size() > 1) {
    ros::Time rt1 = q[1]->header.stamp;
    double t1 = rt1.sec + 1e-9 * rt1.nsec;
    if (t1 >= time) {
      ros::Time rt0 = q[0]->header.stamp;
      double t0 = rt0.sec + 1e-9 * rt0.nsec;

      double d1 = time - t0;
      double d2 = t1 - time;

      // if the later time is closer, erase the earlier
      if (d2 <= d1) {
        q.pop_front();
      }

      return true;
    }
    q.pop_front();
  }

  // should be unreachable
  return false;
}