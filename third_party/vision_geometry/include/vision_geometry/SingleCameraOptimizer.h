#ifndef SINGLE_CAMERA_OPTIMIZER_H
#define SINGLE_CAMERA_OPTIMIZER_H

#include "CameraIntrinsics.h"
#include "CameraOptimizationParameters.h"
#include "DistortionModel.h"

#include <Eigen/Dense>
#include <vector>

class RigidTrans;

class SingleCameraOptimizer {
public:
    SingleCameraOptimizer(
        CameraOptimizationParameters &cop
        , CameraIntrinsics cameraIntrinsics, Eigen::MatrixXd cb3dh
        , double k1 = 0.0, double k2 = 0.0, double k3 = 0.0
        , double p1 = 0.0, double p2 = 0.0);

    CameraIntrinsics &getKOpt();
    DistortionModel &getDMOpt();

    void update(
        std::vector<Eigen::MatrixXd> camPoints
        , std::vector<RigidTrans> rts);

    void prettyPrint();

protected:
    CameraOptimizationParameters &_cop;
    Eigen::MatrixXd _cb3dh;
    CameraIntrinsics _kOpt;
    DistortionModel _dmOpt;
};

#endif
