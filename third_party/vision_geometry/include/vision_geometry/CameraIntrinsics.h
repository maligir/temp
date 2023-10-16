#ifndef CAMERA_INTRINSICS_H
#define CAMERA_INTRINSICS_H

#include <opencv2/core/core.hpp>

#include <Eigen/Dense>

class CameraIntrinsics {
public:
    CameraIntrinsics(
        double alpha = 200, double beta = 200, double gamma = 0.0
        , double u0 = 320, double v0 = 240);

    void update(
        double alpha = 200, double beta = 200, double gamma = 0.0
        , double u0 = 320, double v0 = 240);

    Eigen::Matrix3d getMat();
    cv::Mat getCVMat();

    double getAlpha();
    double getBeta();
    double getGamma();
    double getU0();
    double getV0();

    double setAlpha(double alpha);
    double setBeta(double beta);
    double setGamma(double gamma);
    double setU0(double u0);
    double setV0(double v0);

    void setEqualTo(CameraIntrinsics k);

protected:
    Eigen::Matrix3d _mat;
};

#endif