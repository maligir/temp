#ifndef CAMERA_OPTIMIZATION_PARAMETERS_H
#define CAMERA_OPTIMIZATION_PARAMETERS_H

#include <utility>

class CameraIntrinsics;

class CameraOptimizationParameters {
public:
    CameraOptimizationParameters(
        double alpha, double beta, double gamma, double u0, double v0
        , bool alphaBetaEqual = false, bool alphaBetaPinned = false
        , bool gammaPinned = false , bool u0V0Pinned = false);

    CameraOptimizationParameters(
        CameraIntrinsics k
        , bool alphaBetaEqual = false, bool alphaBetaPinned = false
        , bool gammaPinned = false , bool u0V0Pinned = false);

    void setAlphaBetaEqual(bool val);
    void setAlphaBetaPinned(bool val);
    void setGammaPinned(bool val);
    void setU0V0Pinned(bool val);

    void setAlpha(double val);
    void setBeta(double val);
    void setGamma(double val);
    void setU0V0(double u0, double v0);

    bool getAlphaBetaEqual() const;
    bool getAlphaBetaPinned() const;
    bool getGammaPinned() const;
    bool getU0V0Pinned() const;

    double getAlpha() const;
    double getBeta() const;
    double getGamma() const;
    std::pair<double, double> getU0V0() const;

protected:
    double _alpha, _beta, _gamma, _u0, _v0;
    bool _alphaBetaEqual, _alphaBetaPinned, _gammaPinned, _u0V0Pinned;
};

#endif
