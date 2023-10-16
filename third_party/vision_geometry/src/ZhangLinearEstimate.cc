#include <vision_geometry/ZhangLinearEstimate.h>
#include <vision_geometry/LinearAlgebraShortcuts.h>

#include <iostream>

using namespace Eigen;
using namespace std;

ZhangLinearEstimate::ZhangLinearEstimate(MatrixXd zhangB) : _k(3,3) {
    init(zhangB);
}

ZhangLinearEstimate::ZhangLinearEstimate(vector<MatrixXd> homographies) :
    _k(3,3) {
    cout << "ZhangLinearEstimate::ZhangLinearEstimate:  " << homographies.size()
        << endl;
    MatrixXd A = zhangHomographyConstraints(homographies);
    MatrixXd rns = rightNullSpace(A);
    init(rns);
}

void ZhangLinearEstimate::prettyPrint() {
    cout << "alpha: " << getAlpha() << endl;
    cout << "beta: " << getBeta() << endl;
    cout << "gamma: " << getGamma() << endl;
    cout << "u0: " << getU0() << endl;
    cout << "v0: " << getV0() << endl;
    cout << "lambda: " << _lambda << endl;
}

double ZhangLinearEstimate::getAlpha()      {   return _k.getAlpha();   }
double ZhangLinearEstimate::getBeta()       {   return _k.getBeta();    }
double ZhangLinearEstimate::getGamma()      {   return _k.getGamma();   }
double ZhangLinearEstimate::getU0()         {   return _k.getU0();      }
double ZhangLinearEstimate::getV0()         {   return _k.getV0();      }
double ZhangLinearEstimate::getLamdba()     {   return _lambda; }

CameraIntrinsics ZhangLinearEstimate::getCameraIntrinsics() {
    return _k;
}

void ZhangLinearEstimate::init(Eigen::MatrixXd zhangB) {
    double B11 = zhangB(0,0);
    double B12 = zhangB(1,0);
    double B22 = zhangB(2,0);
    double B13 = zhangB(3,0);
    double B23 = zhangB(4,0);
    double B33 = zhangB(5,0);

    double v0 = (B12 * B13 - B11 * B23) / (B11 * B22 - B12 * B12);
    _lambda = B33 - (B13 * B13 + v0 * (B12 * B13 - B11 * B23)) / B11;
    double alpha = sqrt(_lambda / B11);
    double beta = sqrt(_lambda * B11 / (B11 * B22 - B12 * B12));
    double gamma = -B12 * alpha * alpha * beta / _lambda;
    double u0 = gamma * v0 / beta - B13 * alpha * alpha / _lambda;

    _k.setAlpha(alpha);
    _k.setBeta(beta);
    _k.setGamma(gamma);
    _k.setU0(u0);
    _k.setV0(v0);
}

MatrixXd ZhangLinearEstimate::vIJ(MatrixXd homography, int i, int j) {
    double hi1 = homography(0, i);
    double hj1 = homography(0, j);
    double hi2 = homography(1, i);
    double hj2 = homography(1, j);
    double hi3 = homography(2, i);
    double hj3 = homography(2, j);

    MatrixXd v(1,6);
    v(0,0) = hi1 * hj1;
    v(0,1) = hi1 * hj2 + hi2 * hj1;
    v(0,2) = hi2 * hj2;
    v(0,3) = hi3 * hj1 + hi1 * hj3;
    v(0,4) = hi3 * hj2 + hi2 * hj3;
    v(0,5) = hi3 * hj3;

    return v;
}

MatrixXd ZhangLinearEstimate::zhangHomographyConstraints(MatrixXd homography) {
    MatrixXd V(2,6);
    V.block(0,0,1,6) = vIJ(homography, 0, 1);
    V.block(1,0,1,6) = vIJ(homography, 0, 0) - vIJ(homography, 1, 1);

    return V;
}

MatrixXd ZhangLinearEstimate::zhangHomographyConstraints(
    vector<MatrixXd> homographies) {
    MatrixXd V(2 * homographies.size(),6);
    for(int i = 0; i < homographies.size(); i++) {
        V.block(i * 2, 0, 2, 6) = zhangHomographyConstraints(homographies[i]);
    }

    return V;
}
