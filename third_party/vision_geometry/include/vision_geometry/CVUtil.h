#ifndef CV_UTIL_H
#define CV_UTIL_H

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <Eigen/Dense>

#include <utility>
#include <vector>

void drawCrosshair(cv::Mat &img, int x, int y, int rad, int thickness
                    , cv::Scalar color);

//Mat3b img         -   Image to draw on
//MatrixXd points   -   Points to draw, Cartesian, 2D
void drawCrosshairs(cv::Mat &img, Eigen::MatrixXd points, int rad, int thickness
                    , cv::Scalar color);

void showPoints(cv::Mat &frameBGR, Eigen::MatrixXd inPts, cv::Scalar &s);

std::vector<cv::Mat> loadImages(std::vector<std::string> files);
void detectCorners(
    cv::Size s
    , std::vector<cv::Mat> inImages
    , std::vector<cv::Mat> &outImages
    , std::vector<Eigen::MatrixXd> &outCorners);

bool detectFrameCorners(
    cv::Size s, cv::Mat inImage, cv::Mat &outImage, Eigen::MatrixXd &outCorners);

#endif
