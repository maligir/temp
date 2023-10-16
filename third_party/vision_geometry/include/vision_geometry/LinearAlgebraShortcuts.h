#ifndef LINEAR_ALGEBRA_SHORTCUTS_H
#define LINEAR_ALGEBRA_SHORTCUTS_H

#include <opencv2/opencv.hpp>

#include <Eigen/Dense>

#include <vector>

using namespace std;

Eigen::MatrixXd rightNullSpace(Eigen::MatrixXd A);
cv::Mat convertToCVMat(Eigen::MatrixXd A);
Eigen::MatrixXd convertToEigenMatF(cv::Mat A);
Eigen::MatrixXd convertToEigenMatD(cv::Mat A);
std::vector<Eigen::MatrixXd> convertToEigenMatF(std::vector<cv::Mat> A);
std::vector<Eigen::MatrixXd> convertToEigenMatD(std::vector<cv::Mat> A);

Eigen::MatrixXd convertToEigenMat(std::vector<std::vector<cv::Point2f> > c);

Eigen::MatrixXd convert2ChanCVMatToEigenMatF(cv::Mat A);
std::vector<Eigen::MatrixXd> convert2ChanCVMatToEigenMatF(
    std::vector<cv::Mat> A);

Eigen::MatrixXd columnNorms(Eigen::MatrixXd A);

#endif
