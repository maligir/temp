#ifndef OPT_RT_DIST_COST_FUNCTOR_H
#define OPT_RT_DIST_COST_FUNCTOR_H

#include "CameraIntrinsics.h"
#include "CameraOptimizationParameters.h"
#include "CVUtil.h"
#include "DistortionModel.h"
#include "LinearAlgebraShortcuts.h"
#include "HomCartShortcuts.h"
#include "RigidTrans.h"
#include "TransformShortcuts.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Dense>
#include <vector>

using namespace cv;
using namespace Eigen;
using namespace std;

struct OptRTDistCostFunctor {
public:
    OptRTDistCostFunctor(
        CameraOptimizationParameters cop,
        MatrixXd modelCBH3D, vector<MatrixXd> imageCB);

    bool operator()(double const* const* parameters, double* residuals) const;

    void showOptCBs(Mat &frameBGRA, MatrixXd testCB, MatrixXd imageCB) const;

protected:
    CameraOptimizationParameters _cop;
    MatrixXd _modelCBH3D;
    vector<MatrixXd> _imageCB;
    double _alpha, _beta, _gamma, _u0, _v0;
    bool _alphaBetaEqual, _alphaBetaPinned, _gammaPinned, _u0V0Pinned;
};

#endif
