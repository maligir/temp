#ifndef DISTORTION_MODEL_H
#define DISTORTION_MODEL_H

#include <opencv2/core/core.hpp>

#include <Eigen/Dense>

#include <map>

class DistortionModel {
public:
    DistortionModel(double k1 = 0.0, double k2 = 0.0, double k3 = 0.0
        , double p1 = 0.0, double p2 = 0.0);
    
    std::pair<double, double> distortXY(double x, double y);
    Eigen::MatrixXd distortXY(Eigen::MatrixXd ptsC);

    double getK1();
    double getK2();
    double getK3();

    double getP1();
    double getP2();

    cv::Mat getDistortionCoeffsCV();

    void update(double k1, double k2, double k3, double p1, double p2);
    
protected:
    double _k1, _k2, _k3, _p1, _p2;
};

#endif
