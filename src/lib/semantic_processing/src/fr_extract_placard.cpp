#include "fr_extract_placard.h"
#include "placard.h"
#include "data_packet.h"
#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>

#include <regex>

FRExtractPlacard::FRExtractPlacard(PlacardDatabase& data,
                                     cv::dnn::Net* east_net,
                                     tesseract::TessBaseAPI* ocr,
                                     float conf_thr,
                                     float nms_thr,
                                     std::regex& regex_pref,
                                     std::regex& regex_gen,
                                     const std::string& img_name):
    _data(data)
    , _east_net(east_net)
    , _ocr(ocr)
    , _conf_thr(conf_thr)
    , _nms_thr(conf_thr)
    , _regex_pref(regex_pref)
    , _regex_gen(regex_gen)
    , _img_name(img_name) {}

void FRExtractPlacard::initializePacket(DataPacket* dp) {
    if (_fr) {
        _fr->initializePacket(dp);
    }
}

void FRExtractPlacard::receiveFrame(DataPacket* dp) {
    Eigen::Affine3f& rgb_to_world = dp->rgb_to_world;
    cv::Mat cloud_img = dp->_mats["PointCloudImg4K"];
    cv::Mat& rgb_img = dp->_mats[_img_name];
    int w = cloud_img.cols;
    int h = cloud_img.rows;
    const int16_t* buffer = reinterpret_cast<const int16_t*>(cloud_img.data);
    for (BoundingBox bb : dp->_boxes) {
        Eigen::Vector3f centroid, normal;
        Eigen::Quaternionf quat;
        cv::Mat warped;
        if (getPlacardPose(buffer, w, h, bb, centroid, normal)) {
          normalToQuaternion(rgb_to_world.rotation() * normal, quat);
          cv::Rect roi;
          bb2roi(bb, roi);
          // cv::Mat detection = rgb_img.clone();
          // cv::rectangle(detection, roi, cv::Scalar(0,0,255), 3);
          // cv::Mat raw_patch = rgb_img(roi);
          warpImage(rgb_img, centroid, normal, warped, roi);

          if (roi.width == 0 || roi.height == 0) {
            continue;
          }
          cv::Mat patch = warped(roi);
          // cv::imwrite("detection.jpg", detection);
          // cv::imwrite("patch.jpg", raw_patch);
          // cv::imwrite("warped_patch.jpg", patch);
          // cv::imshow("detection", detection);
          // cv::imshow("patch", raw_patch);
          // cv::imshow("warped patch", patch);
          cv::Mat padded;
          padPatch(patch, padded);
          // cv::imshow("padded patch", padded);
          std::string label = extractText(padded);
          // std::cout << "in camera frame:\n" << rgb_to_world.matrix() << std::endl;
          // std::cout << "adding placard with label: " << label << " centroid: " << centroid.transpose() << std::endl;
          // std::cout << "placard has global position: " << (rgb_to_world*centroid).transpose() << std::endl;
          Eigen::Vector3f world_centroid = rgb_to_world*centroid;
          Eigen::Vector3f camera_pointing = rgb_to_world.rotation()*Eigen::Vector3f(0,0,1);
          // std::cout << rgb_to_world.matrix()(0,3) << "," << rgb_to_world.matrix()(1,3) << ","
          //           << camera_pointing[0] << "," << camera_pointing[1] << ","
          //           << centroid[0] << "," << centroid[1] << ","
          //           << world_centroid[0] << "," << world_centroid[1] << std::endl;
          // cv::waitKey(0);
          Placard p(rgb_to_world*centroid, quat, label, bb.probability, dp->tframe);
          _data.AddPlacard(p);
        }
    }

    if (_fr) {
        _fr->receiveFrame(dp);
    }
    
}

void FRExtractPlacard::bb2roi(BoundingBox& bb, cv::Rect& roi) {
  size_t minx = static_cast<size_t>(4096*bb.xmin + 0.5);
  size_t maxx = static_cast<size_t>(4096*bb.xmax + 0.5);
  size_t miny = static_cast<size_t>(3072*bb.ymin + 0.5);
  size_t maxy = static_cast<size_t>(3072*bb.ymax + 0.5);

  roi = cv::Rect(minx, miny, maxx - minx, maxy - miny);
}

bool FRExtractPlacard::getPlacardPose(const int16_t* buffer,
                                       int w, int h,
                                       BoundingBox& bb,
                                       Eigen::Vector3f& centroid,
                                       Eigen::Vector3f& normal) {
  size_t minx = bb.xmin*w;
  size_t maxx = bb.xmax*w;
  size_t miny = bb.ymin*h;
  size_t maxy = bb.ymax*h;

  size_t npts = (maxy-miny + 1)*(maxx-minx + 1);
  size_t nsample = std::min(npts, (size_t) 100000);
  size_t frac = npts / nsample;
  // std::cout << "depthcloud sampling 1 point for every " << frac << " points\n";
  
  Eigen::Matrix3Xf points(3, nsample);
  size_t point_count = 0;
  size_t valid_count = 0;
  for (size_t row = miny; row <= maxy; row++) {
    for (size_t col = minx; col <= maxx; col++) {
      if (point_count == nsample) {
        break;
      }
      size_t idx = w*row + col;
      int16_t z = buffer[3 * idx + 2];
      if (z < 250 || z > 2880 || std::isnan(z)) {
        continue;
      }
      valid_count++;
      if (valid_count % frac > 0) {
        continue;
      }
      int16_t x = buffer[3 * idx + 0];
      int16_t y = buffer[3 * idx + 1];
      points.block<3,1>(0,point_count) << static_cast<float>(x)/1000.,
                                    static_cast<float>(y)/1000.,
                                    static_cast<float>(z)/1000.;
      point_count++;
    }
  }
  if (point_count < 100) {
    return false;
  }

  points.conservativeResize(Eigen::NoChange, point_count);
  centroid = points.rowwise().mean();

  Eigen::Matrix3Xf points_centered = points.colwise() - centroid;
  Eigen::BDCSVD<Eigen::Matrix3Xf> svd = points_centered.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
  // std::cout << "point cloud singular values: " << svd.singularValues().transpose() << std::endl;
  // std::cout << "point cloud svd U matrix:\n" << svd.matrixU() << std::endl;
  normal = svd.matrixU().rightCols<1>();
  normal.normalize();

  if (normal.dot(centroid) > 0) {
      normal = -normal;
  }
  return true;
}

void FRExtractPlacard::decode(const cv::Mat& scores,
                               const cv::Mat& geometry,
                               float scoreThresh,
                               std::vector<cv::RotatedRect>& detections,
                               std::vector<float>& confidences) {
    CV_Assert(scores.dims == 4); CV_Assert(geometry.dims == 4); CV_Assert(scores.size[0] == 1);
    CV_Assert(geometry.size[0] == 1); CV_Assert(scores.size[1] == 1); CV_Assert(geometry.size[1] == 5);
    CV_Assert(scores.size[2] == geometry.size[2]); CV_Assert(scores.size[3] == geometry.size[3]);

    const int height = scores.size[2];
    const int width = scores.size[3];
    for (int y = 0; y < height; ++y)
    {
        const float* scoresData = scores.ptr<float>(0, 0, y);
        const float* x0_data = geometry.ptr<float>(0, 0, y);
        const float* x1_data = geometry.ptr<float>(0, 1, y);
        const float* x2_data = geometry.ptr<float>(0, 2, y);
        const float* x3_data = geometry.ptr<float>(0, 3, y);
        const float* anglesData = geometry.ptr<float>(0, 4, y);
        for (int x = 0; x < width; ++x)
        {
            float score = scoresData[x];
            if (score < scoreThresh)
                continue;

            // Decode a prediction.
            // Multiple by 4 because feature maps are 4 time less than input image.
            float offsetX = x * 4.0f, offsetY = y * 4.0f;
            float angle = anglesData[x];
            float cosA = std::cos(angle);
            float sinA = std::sin(angle);
            float h = x0_data[x] + x2_data[x];
            float w = x1_data[x] + x3_data[x];

            cv::Point2f offset(offsetX + cosA * x1_data[x] + sinA * x2_data[x],
                           offsetY - sinA * x1_data[x] + cosA * x2_data[x]);
            cv::Point2f p1 = cv::Point2f(-sinA * h, -cosA * h) + offset;
            cv::Point2f p3 = cv::Point2f(-cosA * w, sinA * w) + offset;
            cv::RotatedRect r(0.5f * (p1 + p3), cv::Size2f(w, h), -angle * 180.0f / (float)CV_PI);
            detections.push_back(r);
            confidences.push_back(score);
        }
    }
}

void FRExtractPlacard::applyEastNet(cv::Mat patch, std::vector<cv::Rect>& text_patches) {
  std::vector<cv::RotatedRect> boxes;
  std::vector<float> confidences;
  {
    cv::Mat blob;
    cv::dnn::blobFromImage(patch, blob, 1.0,
                           cv::Size(patch.cols, patch.rows),
                           cv::Scalar(123.68, 116.78, 103.94),
                           true, false);
    std::vector<std::string> outputLayers(2);
    outputLayers[0] = "feature_fusion/Conv_7/Sigmoid";
    outputLayers[1] = "feature_fusion/concat_3";
    std::vector<cv::Mat> output;
    _east_net->setInput(blob);
    _east_net->forward(output, outputLayers);
    cv::Mat scores = output[0];
    cv::Mat geometry = output[1];
    decode(scores, geometry, _conf_thr, boxes, confidences);
  }

  {
    cv::Mat blob;
    cv::dnn::blobFromImage(cv::Scalar(255, 255, 255) - patch,
                           blob, 1.0,
                           cv::Size(patch.cols, patch.rows),
                           cv::Scalar(123.68, 116.78, 103.94),
                           true, false);
    std::vector<std::string> outputLayers(2);
    outputLayers[0] = "feature_fusion/Conv_7/Sigmoid";
    outputLayers[1] = "feature_fusion/concat_3";
    std::vector<cv::Mat> output;
    _east_net->setInput(blob);
    _east_net->forward(output, outputLayers);
    cv::Mat scores = output[0];
    cv::Mat geometry = output[1];
    decode(scores, geometry, _conf_thr, boxes, confidences);
  }

  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confidences, _conf_thr, _nms_thr, indices);

  for (size_t i = 0; i < indices.size(); ++i) {
    cv::RotatedRect& box = boxes[indices[i]];
    cv::Rect bbox = box.boundingRect();
    bbox.x = 0;
    bbox.width = patch.cols;
    // bbox.y -= 0.25*bbox.height;
    // bbox.height *= 1.5;
    if (bbox.height + bbox.y > patch.rows) {
      bbox.height = patch.rows - bbox.y;
    }
    bbox.y = std::max(0, bbox.y);
    text_patches.push_back(bbox);
  }
}

std::string FRExtractPlacard::applyTesseract(cv::Mat patch) {
  _ocr->SetImage(patch.data, patch.cols, patch.rows, 1, patch.step);
  std::string outText = std::string(_ocr->GetUTF8Text());
  outText.erase(std::remove_if(outText.begin(), outText.end(), ::isspace), outText.end());
  return outText;
}

void FRExtractPlacard::padPatch(cv::Mat& patch, cv::Mat& padded) {
  int col_r = patch.cols % 32;
  int row_r = patch.rows % 32;
  int col_pad = col_r > 0 ? 32-col_r : 0;
  int row_pad = row_r > 0 ? 32-row_r : 0;

  cv::copyMakeBorder(patch,padded,(int) (row_pad+1)/2,(int) row_pad/2,(int) (col_pad+1)/2,(int) col_pad/2,cv::BORDER_CONSTANT,cv::Scalar(0,0,0));
}

void FRExtractPlacard::warpImage(cv::Mat& img_raw,
                                  Eigen::Vector3f centroid,
                                  Eigen::Vector3f normal,
                                  cv::Mat& warped,
                                  cv::Rect& roi) {
    const float mult = 1;
    Eigen::Vector3f adjoint(normal.z(), 0, -normal.x());
    Eigen::Vector3f vertical(0, 1, 0);
    adjoint *= mult * 0.151;
    vertical *= mult * 0.051;
    float fx = 1959.466552734375;
    float fy = 1959.18603515625;
    float cx = 2043.764404296875;
    float cy = 1559.04150390625;
    Eigen::Matrix3f K = Eigen::MatrixXf::Identity(3,3);
    K(0,0) = fx;
    K(1,1) = fy;
    K(0,2) = cx;
    K(1,2) = cy;
    Eigen::Matrix<float, 3, 4> eig_pts;

    if (adjoint.x()*centroid.z() < adjoint.z()*centroid.x()
          && centroid.z() - adjoint.z() > 0
          && centroid.z() + adjoint.z() > 0) {
        eig_pts.block<3,1>(0,0) = centroid + adjoint - vertical;
        eig_pts.block<3,1>(0,1) = centroid - adjoint - vertical;
        eig_pts.block<3,1>(0,2) = centroid + adjoint + vertical;
        eig_pts.block<3,1>(0,3) = centroid - adjoint + vertical;
    } else {
        eig_pts.block<3,1>(0,0) = centroid - adjoint - vertical;
        eig_pts.block<3,1>(0,1) = centroid + adjoint - vertical;
        eig_pts.block<3,1>(0,2) = centroid - adjoint + vertical;
        eig_pts.block<3,1>(0,3) = centroid + adjoint + vertical;
    }

    eig_pts.block<3,1>(0,0) /= eig_pts(2,0); // top-left
    eig_pts.block<3,1>(0,1) /= eig_pts(2,1); // top-right
    eig_pts.block<3,1>(0,2) /= eig_pts(2,2); // bottom-left
    eig_pts.block<3,1>(0,3) /= eig_pts(2,3); // bottom-right

    eig_pts = K * eig_pts;
    Eigen::Vector3f min_coords = eig_pts.array().rowwise().minCoeff();
    Eigen::Vector3f max_coords = eig_pts.array().rowwise().maxCoeff();
    Eigen::Vector3f span = max_coords - min_coords;

    cv::Point2f src[4];
    cv::Point2f dst[4];

    for (int i = 0; i < 4; i++) {
        src[i] = cv::Point2f(eig_pts(0,i), eig_pts(1,i));
    }
    dst[0] = cv::Point2f(0,0);
    dst[1] = cv::Point2f(span.x(),0);
    dst[2] = cv::Point2f(0,span.y());
    dst[3] = cv::Point2f(span.x(),span.y());

    cv::Mat trans = cv::getPerspectiveTransform(src, dst);

    cv::Mat cv_span(cv::Size(2, 3), trans.type());
    cv_span.at<double>(0,0) = (double) roi.x;
    cv_span.at<double>(1,0) = (double) roi.y;
    cv_span.at<double>(2,0) = (double) 1.0;
    cv_span.at<double>(0,1) = (double) roi.x + roi.width;
    cv_span.at<double>(1,1) = (double) roi.y + roi.height;
    cv_span.at<double>(2,1) = (double) 1.0;
    cv::Mat new_cv_span = trans * cv_span;

    int roi_minx = std::max(static_cast<int>(new_cv_span.at<double>(0,0) / new_cv_span.at<double>(2,0) + 0.5), 0);
    int roi_miny = std::max(static_cast<int>(new_cv_span.at<double>(1,0) / new_cv_span.at<double>(2,0) + 0.5), 0);
    int roi_maxx = std::min(static_cast<int>(new_cv_span.at<double>(0,1) / new_cv_span.at<double>(2,1) + 0.5), 4096);
    int roi_maxy = std::min(static_cast<int>(new_cv_span.at<double>(1,1) / new_cv_span.at<double>(2,1) + 0.5), 3072);

    roi = cv::Rect(roi_minx, roi_miny, roi_maxx - roi_minx, roi_maxy - roi_miny);
    if (roi_maxy < roi_miny || roi_maxx < roi_minx) {
      roi.x = 0;
      roi.y = 0;
      roi.width = (int) span.x() + 0.5;
      roi.height = (int) span.y() + 0.5;
    } 

    // cv::Size s(1.1*span.x()+0.5, 1.1*span.y()+0.5);
    cv::Size s(std::max(roi.x+roi.width,(int) (span.x()+0.5)),
               std::max(roi.y+roi.height,(int) (span.y()+0.5)));
    cv::warpPerspective(img_raw, warped, trans, s, cv::INTER_LINEAR);
    cv::cvtColor(warped, warped, cv::COLOR_BGRA2BGR);
}

void FRExtractPlacard::normalToQuaternion(const Eigen::Vector3f& normal,
                                           Eigen::Quaternionf& quat) {
  Eigen::Vector3f n(normal.x(), normal.y(), 0);
  n.normalize();
  Eigen::Matrix3f R;
  R.block<3,1>(0,0) = n;
  R.block<3,1>(0,1) = Eigen::Vector3f(-n.y(),
                                      n.x(),
                                      0);
  R.block<3,1>(0,2) = Eigen::Vector3f(0, 0, 1);
  
  quat = Eigen::Quaternionf(R);
}

std::string FRExtractPlacard::extractText(cv::Mat& warped) {
  std::vector<cv::Rect> text_patches;
  applyEastNet(warped, text_patches);
  // text_patches.push_back(cv::Rect(0,0,warped.cols,warped.rows));
  cv::Mat text_lines = warped.clone();
  std::vector<std::string> labels;
  int patch_count = 0;
  for (cv::Rect patch : text_patches) {
    patch_count++;
    cv::rectangle(text_lines, patch, cv::Scalar(0,0,255));
    cv::Mat grey;
    cvtColor(warped, grey, cv::COLOR_BGR2GRAY);
    double maxValue = 255;
    for (double thresh = 5; thresh < 255; thresh += 5) {
      cv::Mat dst;
      threshold(grey, dst, thresh, maxValue, cv::THRESH_BINARY_INV);
      // std::string label1 = "bw_patch" + std::to_string(patch_count) + "_thr" + std::to_string(thresh);
      // std::string label2 = label1 + "_inv.jpg";
      // label1 += ".jpg";
      // cv::imwrite(label1, dst(patch)); 
      // cv::imwrite(label2, 255 - dst(patch));
      std::string l1 = applyTesseract(dst(patch));
      std::string l2 = applyTesseract(255 - dst(patch));
      labels.push_back(l1);
      labels.push_back(l2);
    }
  }
  // cv::imwrite("text_lines.jpg", text_lines);
  // cv::imshow("text lines", text_lines);

  std::string outText = "";
  for (std::string label : labels) {
    std::smatch match;
    if (std::regex_search(label, match, _regex_pref)) {
      return match[0];
    } else if (std::regex_search(label, match, _regex_gen)) {
      outText = match[0];
    }
  }

  return outText;
}