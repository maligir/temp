#ifndef ZHANG_LINEAR_ESTIMATE_H
#define ZHANG_LINEAR_ESTIMATE_H

#include "CameraIntrinsics.h"

#include <Eigen/Dense>
#include <vector>

class ZhangLinearEstimate {
public:
    ZhangLinearEstimate(Eigen::MatrixXd zhangB);
    ZhangLinearEstimate(std::vector<Eigen::MatrixXd> homographies);

    void prettyPrint();

    double getAlpha();
    double getBeta();
    double getGamma();
    double getU0();
    double getV0();
    double getLamdba();

    CameraIntrinsics getCameraIntrinsics();

    static Eigen::MatrixXd vIJ(Eigen::MatrixXd homography, int i, int j);
    static Eigen::MatrixXd zhangHomographyConstraints(
        Eigen::MatrixXd homography);
    static Eigen::MatrixXd zhangHomographyConstraints(
        std::vector<Eigen::MatrixXd> homographies);

protected:
    void init(Eigen::MatrixXd zhangB);

    double _lambda;
    CameraIntrinsics _k;
};

#endif
