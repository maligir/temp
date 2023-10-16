#include <vision_geometry/DistortionModel.h>

using namespace cv;
using namespace Eigen;
using namespace std;

DistortionModel::DistortionModel(double k1, double k2, double k3
    , double p1, double p2) :
    _k1(k1), _k2(k2), _k3(k3), _p1(p1), _p2(p2) {}

pair<double, double> DistortionModel::distortXY(double x, double y) {
    pair<double, double>xy;
    double r2 = x * x + y * y;
    double r4 = r2 * r2;
    double r6 = r4 * r2;
    double radialTerm = _k1 * r2 + _k2 * r4 + _k3 * r6;
    double xTanTerm = 2.0 * _p1 * x * y + _p2 * (r2 + 2.0 * x * x);
    double yTanTerm = _p1 * (r2 + 2.0 * y * y) + 2.0 * _p2 * x * y;
    xy.first = x + x * radialTerm + xTanTerm;
    xy.first = y + y * radialTerm + yTanTerm;
    return xy;
}

MatrixXd DistortionModel::distortXY(MatrixXd ptsC) {
    MatrixXd ptsD(2,ptsC.cols());
    for(size_t i = 0; i < ptsC.cols(); i++) {
        double x = ptsC(0,i);
        double y = ptsC(1,i);
        double r2 = x * x + y * y;
        double r4 = r2 * r2;
        double r6 = r4 * r2;
        double radialTerm = _k1 * r2 + _k2 * r4 + _k3 * r6;
        double xTanTerm = 2.0 * _p1 * x * y + _p2 * (r2 + 2.0 * x * x);
        double yTanTerm = _p1 * (r2 + 2.0 * y * y) + 2.0 * _p2 * x * y;
        ptsD(0,i) = x + x * radialTerm + xTanTerm;
        ptsD(1,i) = y + y * radialTerm + yTanTerm;
    }
    return ptsD;
}

double DistortionModel::getK1() {   return _k1; }
double DistortionModel::getK2() {   return _k2; }
double DistortionModel::getK3() {   return _k3; }

double DistortionModel::getP1() {   return _p1; }
double DistortionModel::getP2() {   return _p2; }

Mat DistortionModel::getDistortionCoeffsCV() {
    Mat retVal(1,5,CV_32F);
    retVal.at<float>(0,0) = _k1;
    retVal.at<float>(0,1) = _k2;
    retVal.at<float>(0,2) = _p1;
    retVal.at<float>(0,3) = _p2;
    retVal.at<float>(0,4) = _k3;
    return retVal;
}

void DistortionModel::update(double k1, double k2, double k3
    , double p1, double p2) {
    _k1 = k1;
    _k2 = k2;
    _k3 = k3;

    _p1 = p1;
    _p2 = p2;
}
