#include <vision_geometry/TransformShortcuts.h>
#include <vision_geometry/CameraIntrinsics.h>
#include <vision_geometry/DistortionModel.h>
#include <vision_geometry/HomCartShortcuts.h>
#include <vision_geometry/LinearAlgebraShortcuts.h>
#include <vision_geometry/RigidTrans.h>

#include "opencv2/opencv.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <cmath>

using namespace cv;
using namespace Eigen;
using namespace std;

MatrixXd rotMat(MatrixXd inRotVect) {
    Mat cvRMat(3, 3, CV_64F);
    Mat cvRVect = convertToCVMat(inRotVect);
    Rodrigues(cvRVect, cvRMat);
    return convertToEigenMatD(cvRMat);
}

MatrixXd rotVect(MatrixXd inRotMat) {
    Mat cvRMat = convertToCVMat(inRotMat);
    Mat cvRVect(3, 1, CV_64F);
    Rodrigues(cvRMat, cvRVect);
    return convertToEigenMatD(cvRVect);
}

MatrixXd positiveBoundedRotVect(MatrixXd inRotVect) {
    double n = inRotVect.norm();
    MatrixXd outRotVect = inRotVect / n;
    double twoMPi = 2.0 * M_PI;
    if(inRotVect(0,0) < 0) {
        outRotVect = -outRotVect;
        n = twoMPi - n;
    }
    while(n >= twoMPi) n -= twoMPi;
    while(n <= 0) n += twoMPi;
    return outRotVect * n;
}

MatrixXd projectPts(
    CameraIntrinsics k, DistortionModel dm, RigidTrans rts, MatrixXd pts3DH) {
    MatrixXd camPoints(2, pts3DH.cols());
    camPoints =
        divideByLastRowRemoveLastRow(
            k.getMat() *
            addARowOfConst(dm.distortXY(
                divideByLastRowRemoveLastRow(
                    rts.getIdealProjection() * pts3DH)), 1.0));
    return camPoints;
}

vector<MatrixXd> projectPts(
    CameraIntrinsics k, DistortionModel dm, vector<RigidTrans> rts
    , MatrixXd pts3DH) {
    vector<MatrixXd> camPoints;
    for(int i = 0; i < rts.size(); i++)
        camPoints.push_back(projectPts(k, dm, rts[i], pts3DH));
    return camPoints;
}

vector<MatrixXd> projectHPtsToC(vector<MatrixXd> projs, MatrixXd ptsH) {
    vector<MatrixXd> retVal;
    for(size_t i = 0; i < projs.size(); i++) {
        retVal.push_back(divideByLastRowRemoveLastRow(projs[i] * ptsH));
    }
    return retVal;
}
