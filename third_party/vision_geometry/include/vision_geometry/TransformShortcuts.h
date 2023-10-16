#ifndef TRANSFORM_SHORTCUTS_H
#define TRANSFORM_SHORTCUTS_H

#include <Eigen/Dense>

#include <vector>

class CameraIntrinsics;
class DistortionModel;
class RigidTrans;

Eigen::MatrixXd rotMat(Eigen::MatrixXd inRotVect);
Eigen::MatrixXd rotVect(Eigen::MatrixXd inRotMat);
Eigen::MatrixXd positiveBoundedRotVect(Eigen::MatrixXd inRotVect);

Eigen::MatrixXd projectPts(
    CameraIntrinsics k, DistortionModel dm, RigidTrans rts
    , Eigen::MatrixXd pts3DH);
std::vector<Eigen::MatrixXd> projectPts(
    CameraIntrinsics k, DistortionModel dm, std::vector<RigidTrans> rts
    , Eigen::MatrixXd pts3DH);

std::vector<Eigen::MatrixXd> projectHPtsToC(
    std::vector<Eigen::MatrixXd> homographies, Eigen::MatrixXd ptsH);

#endif
