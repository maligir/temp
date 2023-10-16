#include <vision_geometry/OptRTDistCostFunctor.h>

OptRTDistCostFunctor::OptRTDistCostFunctor(
    CameraOptimizationParameters cop,
    MatrixXd modelCBH3D, vector<MatrixXd> imageCB) :
    _cop(cop)
    , _modelCBH3D(modelCBH3D), _imageCB(imageCB)
    , _alphaBetaEqual(false), _alphaBetaPinned(false), _gammaPinned(false)
    , _u0V0Pinned(false)
    {}

bool OptRTDistCostFunctor::operator()(
    double const* const* parameters, double* residuals) const {
    double alpha = _cop.getAlphaBetaPinned()?_cop.getAlpha() : parameters[0][0];
    double beta = _cop.getAlphaBetaPinned()?_cop.getBeta() : parameters[0][1];
    if(_cop.getAlphaBetaEqual()) alpha = beta;

    double gamma = _cop.getGammaPinned() ? _cop.getGamma() : parameters[0][2];

    double u0 = parameters[0][3];
    double v0 = parameters[0][4];

    if(_cop.getU0V0Pinned()) {
        pair<double, double> u0v0 = _cop.getU0V0();
        u0 = u0v0.first;
        v0 = u0v0.second;
    }

    //cout << "U0:    " << u0 << endl;
    //cout << "V0:    " << v0 << endl;

    CameraIntrinsics k(alpha, beta, gamma, u0, v0);
    DistortionModel dm(parameters[0][5], parameters[0][6]);

    Mat frameBGRA(480, 640, CV_8UC4, Scalar(0, 0, 0, 255));

    size_t ctr = 0;
    //for(size_t i = 0; i < 1; i++) {
    for(size_t i = 0; i < _imageCB.size(); i++) {
        RigidTrans rt(
            parameters[i+1][0], parameters[i+1][1], parameters[i+1][2]
            , parameters[i+1][3], parameters[i+1][4], parameters[i+1][5]);
        MatrixXd testCB = projectPts(k, dm, rt, _modelCBH3D);
        
        //showOptCBs(frameBGRA, testCB, _imageCB[i]);

        testCB -= _imageCB[i];
        MatrixXd vals = columnNorms(testCB);
        for(size_t col = 0; col < vals.cols(); col++)
            residuals[ctr++] = vals(0, col);
    }
    //waitKey(1);

    return true;
}

void OptRTDistCostFunctor::showOptCBs(
    Mat &frameBGRA, MatrixXd testCB, MatrixXd imageCB) const {
    Scalar colorA(0,255,255, 127);
    Scalar colorB(255,255,0, 127);
    showPoints(frameBGRA, testCB, colorA);
    showPoints(frameBGRA, imageCB, colorB);
}
