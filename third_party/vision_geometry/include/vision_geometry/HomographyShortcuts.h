#ifndef HOMOGRAPHY_SHORTCUTS_H
#define HOMOGRAPHY_SHORTCUTS_H

#include "CameraIntrinsics.h"
#include "ModelChessboard.h"
#include "RigidTrans.h"

#include <Eigen/Dense>
#include <vector>

Eigen::MatrixXd homographyDLTConstraint(Eigen::MatrixXd x,
                                        Eigen::MatrixXd xPrime);
Eigen::MatrixXd homographyDLT(Eigen::MatrixXd x, Eigen::MatrixXd xPrime);
Eigen::MatrixXd computeDLTHomography(Eigen::MatrixXd x, Eigen::MatrixXd xPrime);

std::vector<Eigen::MatrixXd> computeHomographies(
    Eigen::MatrixXd modelPoints, std::vector<Eigen::MatrixXd> camPoints);

Eigen::MatrixXd optimizeHomography(Eigen::MatrixXd modelPts,
                                   Eigen::MatrixXd imagePts,
                                   Eigen::MatrixXd homography);
std::vector<Eigen::MatrixXd> optimizeHomographies(
    Eigen::MatrixXd modelPts, std::vector<Eigen::MatrixXd> imagePts,
    std::vector<Eigen::MatrixXd> homography);

RigidTrans optimizeRT(CameraIntrinsics &intrinsics,
                      const Eigen::MatrixXd &image_corners,
                      const std::vector<Eigen::Vector3d> &world_corners,
                      RigidTrans &rt);

std::vector<RigidTrans> extractRTs(CameraIntrinsics k,
                                   std::vector<Eigen::MatrixXd> homographies);

#endif
