#include <vision_geometry/CameraIntrinsics.h>
#include <vision_geometry/LinearAlgebraShortcuts.h>

using namespace cv;
using namespace Eigen;

CameraIntrinsics::CameraIntrinsics(
    double alpha, double beta, double gamma, double u0, double v0) :
    _mat(Matrix3d::Identity()) {
    update(alpha, beta, gamma, u0, v0);
}

void CameraIntrinsics::update(
    double alpha, double beta, double gamma, double u0, double v0) {
    _mat(0,0) = alpha;
    _mat(1,1) = beta;
    _mat(0,1) = gamma;
    _mat(0,2) = u0;
    _mat(1,2) = v0;
}

Matrix3d CameraIntrinsics::getMat() {
    return _mat;
}

Mat CameraIntrinsics::getCVMat() {
    return convertToCVMat(_mat);
}

double CameraIntrinsics::getAlpha() {   return _mat(0,0);   }
double CameraIntrinsics::getBeta()  {   return _mat(1,1);   }
double CameraIntrinsics::getGamma() {   return _mat(0,1);   }
double CameraIntrinsics::getU0()    {   return _mat(0,2);   }
double CameraIntrinsics::getV0()    {   return _mat(1,2);   }

double CameraIntrinsics::setAlpha(double alpha) {   _mat(0,0) = alpha;  }
double CameraIntrinsics::setBeta(double beta)   {   _mat(1,1) = beta;   }
double CameraIntrinsics::setGamma(double gamma) {   _mat(0,1) = gamma;  }
double CameraIntrinsics::setU0(double u0)       {   _mat(0,2) = u0;     }
double CameraIntrinsics::setV0(double v0)       {   _mat(1,2) = v0;     }

void CameraIntrinsics::setEqualTo(CameraIntrinsics k) {
    setAlpha(k.getAlpha());
    setBeta(k.getBeta());
    setGamma(k.getGamma());
    setU0(k.getU0());
    setV0(k.getV0());
}
