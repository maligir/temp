#include <vision_geometry/SingleCameraOptimizer.h>
#include <vision_geometry/OptRTDistCostFunctor.h>

#include "ceres/ceres.h"

using namespace ceres;
using namespace Eigen;
using namespace std;

SingleCameraOptimizer::SingleCameraOptimizer(
    CameraOptimizationParameters &cop
    , CameraIntrinsics cameraIntrinsics, MatrixXd cb3dh
    , double k1, double k2, double k3, double p1, double p2) :
    _cop(cop), _cb3dh(cb3dh) {
    _kOpt.setEqualTo(cameraIntrinsics);
    _dmOpt.update(k1, k2, k3, p1, p2);
}

CameraIntrinsics &SingleCameraOptimizer::getKOpt()  {   return _kOpt;   }
DistortionModel &SingleCameraOptimizer::getDMOpt()  {   return _dmOpt;  }

//MatrixXd cb3dh: chessboard.getModelCBH3D()
void SingleCameraOptimizer::update(
    vector<MatrixXd> camPoints
    , vector<RigidTrans> rts) {
    Problem problem;
    OptRTDistCostFunctor *optRTDistCostFunctor =
        new OptRTDistCostFunctor(_cop, _cb3dh, camPoints);
    DynamicNumericDiffCostFunction<OptRTDistCostFunctor> *cost_function =
      new DynamicNumericDiffCostFunction<OptRTDistCostFunctor>(
        optRTDistCostFunctor);

    cost_function->AddParameterBlock(10);
    vector<double *>parameters;
    parameters.push_back(new double[10]);
    parameters[0][0] = _kOpt.getAlpha();
    parameters[0][1] = _kOpt.getBeta();
    parameters[0][2] = _kOpt.getGamma();
    parameters[0][3] = _kOpt.getU0();
    parameters[0][4] = _kOpt.getV0();
    parameters[0][5] = _dmOpt.getK1();
    parameters[0][6] = _dmOpt.getK2();
    parameters[0][7] = _dmOpt.getK3();
    parameters[0][8] = _dmOpt.getP1();
    parameters[0][9] = _dmOpt.getP2();

    for(size_t i = 0; i < rts.size(); i++) {
        cost_function->AddParameterBlock(6);
        parameters.push_back(new double[6]);
        parameters[i + 1][0] = rts[i].getTransVect()(0,0);
        parameters[i + 1][1] = rts[i].getTransVect()(1,0);
        parameters[i + 1][2] = rts[i].getTransVect()(2,0);
        parameters[i + 1][3] = rts[i].getRotVect()(0,0);
        parameters[i + 1][4] = rts[i].getRotVect()(1,0);
        parameters[i + 1][5] = rts[i].getRotVect()(2,0);
    }

    cost_function->SetNumResiduals(rts.size() * _cb3dh.cols());
    problem.AddResidualBlock(cost_function, NULL, parameters);

    Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    cout << summary.BriefReport() << endl;

    double alpha = parameters[0][0];
    double beta = parameters[0][1];
    if(_cop.getAlphaBetaEqual()) alpha = beta;
    if(_cop.getAlphaBetaPinned()) {
        alpha = _cop.getAlpha();
        beta = _cop.getBeta();
    }

    double gamma = _cop.getGammaPinned() ? _cop.getGamma() : parameters[0][2];

    double u0 = parameters[0][3];
    double v0 = parameters[0][4];
    if(_cop.getU0V0Pinned()) {
        pair<double, double> u0v0 = _cop.getU0V0();
        u0 = u0v0.first;
        v0 = u0v0.second;
    }

    _kOpt.update(alpha, beta, gamma, u0, v0);
    _dmOpt.update(parameters[0][5], parameters[0][6], parameters[0][7]
        , parameters[0][8], parameters[0][9]);

    for(size_t i = 0; i <= rts.size(); i++) {
        delete[] parameters[i];
    }
}

void SingleCameraOptimizer::prettyPrint() {
    cout << "kOpt:  " << endl << _kOpt.getMat() << endl;
    cout << "dmOpt:  " <<
        _dmOpt.getK1() << "    " <<
        _dmOpt.getK2() << "    " <<
        _dmOpt.getK3() << "    " <<
        _dmOpt.getP1() << "    " <<
        _dmOpt.getP2() <<
        endl;
}
