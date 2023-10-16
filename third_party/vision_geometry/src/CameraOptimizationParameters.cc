#include <vision_geometry/CameraOptimizationParameters.h>
#include <vision_geometry/CameraIntrinsics.h>

using namespace std;

CameraOptimizationParameters::CameraOptimizationParameters(
    double alpha, double beta, double gamma, double u0, double v0
    , bool alphaBetaEqual, bool alphaBetaPinned, bool gammaPinned
    , bool u0V0Pinned) :
    _alpha(alpha)
    , _beta(beta)
    , _gamma(gamma)
    , _u0(u0)
    , _v0(v0)

    , _alphaBetaEqual(alphaBetaEqual)
    , _alphaBetaPinned(alphaBetaPinned)
    , _gammaPinned(gammaPinned)
    , _u0V0Pinned(u0V0Pinned)
    {}

CameraOptimizationParameters::CameraOptimizationParameters(
    CameraIntrinsics k
    , bool alphaBetaEqual, bool alphaBetaPinned
    , bool gammaPinned , bool u0V0Pinned) :
    _alpha(k.getAlpha())
    , _beta(k.getBeta())
    , _gamma(k.getGamma())
    , _u0(k.getU0())
    , _v0(k.getV0())

    , _alphaBetaEqual(alphaBetaEqual)
    , _alphaBetaPinned(alphaBetaPinned)
    , _gammaPinned(gammaPinned)
    , _u0V0Pinned(u0V0Pinned)
    {}

void CameraOptimizationParameters::setAlphaBetaEqual(bool val) {
    _alphaBetaEqual = val;
}

void CameraOptimizationParameters::setAlphaBetaPinned(bool val) {
    _alphaBetaPinned = val;
}

void CameraOptimizationParameters::setGammaPinned(bool val) {
    _gammaPinned = val;
}

void CameraOptimizationParameters::setU0V0Pinned(bool val) {
    _u0V0Pinned = val;
}

void CameraOptimizationParameters::setAlpha(double val) {
    _alpha = val;
}

void CameraOptimizationParameters::setBeta(double val) {
    _beta = val;
}

void CameraOptimizationParameters::setGamma(double val) {
    _gamma = val;
}

void CameraOptimizationParameters::setU0V0(double u0, double v0) {
    _u0 = u0;
    _v0 = v0;
}

bool CameraOptimizationParameters::getAlphaBetaEqual() const {
    return _alphaBetaEqual;
}

bool CameraOptimizationParameters::getAlphaBetaPinned() const {
    return _alphaBetaPinned;
}

bool CameraOptimizationParameters::getGammaPinned() const {
    return _gammaPinned;
}

bool CameraOptimizationParameters::getU0V0Pinned() const {
    return _u0V0Pinned;
}

double CameraOptimizationParameters::getAlpha() const {
    return _alpha;
}

double CameraOptimizationParameters::getBeta() const {
    return _beta;
}

double CameraOptimizationParameters::getGamma() const {
    return _gamma;
}

pair<double, double> CameraOptimizationParameters::getU0V0() const {
    return pair<double, double>(_u0,_v0);
}

