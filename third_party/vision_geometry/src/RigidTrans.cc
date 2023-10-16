#include <vision_geometry/RigidTrans.h>
#include <vision_geometry/TransformShortcuts.h>
#include <vision_geometry/Util.h>

#include <iostream>

using namespace Eigen;
using namespace std;

RigidTrans::RigidTrans(MatrixXd rotVect, MatrixXd transVect) :
    _rotVect(rotVect), _transVect(transVect), _idealProj(3,4)
    , _transMat(Matrix4d::Identity()) {
    computeTransMatAndIdealProj();
}

RigidTrans::RigidTrans(
    double tX, double tY, double tZ
    , double rX, double rY, double rZ, double rTheta) :
    _rotVect(3,1), _transVect(3,1), _idealProj(3,4)
    , _transMat(Matrix4d::Identity()) {
    init(tX, tY, tZ, rX * rTheta, rY * rTheta, rZ * rTheta);
}

RigidTrans::RigidTrans(
    double tX, double tY, double tZ
    , double rX, double rY, double rZ) :
    _rotVect(3,1), _transVect(3,1), _idealProj(3,4)
    , _transMat(Matrix4d::Identity()) {
    init(tX, tY, tZ, rX, rY, rZ);
}

void RigidTrans::init(
    double tX, double tY, double tZ
    , double rX, double rY, double rZ) {
    _transVect(0,0) = tX;
    _transVect(1,0) = tY;
    _transVect(2,0) = tZ;
    _rotVect(0,0) = rX;
    _rotVect(1,0) = rY;
    _rotVect(2,0) = rZ;
    _rotVect = positiveBoundedRotVect(_rotVect);
    computeTransMatAndIdealProj();
}

void RigidTrans::computeTransMatAndIdealProj() {
    _transMat.block(0, 0, 3, 3) = rotMat(_rotVect);
    _transMat.block(0, 3, 3, 1) = _transVect;
    _idealProj = _transMat.block(0,0,3,4);
}

Quaternion<double> RigidTrans::getRotationQuat() {
    Matrix3d rotationMatrix = getRotationMat();
    Quaterniond q(rotationMatrix);
    return q;
}

Matrix3d RigidTrans::getRotationMat() {
    MatrixXd mat = rotMat(_rotVect);
    Matrix3d rotationMatrix;
    rotationMatrix << mat(0, 0), mat(0, 1), mat(0, 2),
                      mat(1, 0), mat(1, 1), mat(1, 2),
                      mat(2, 0), mat(2, 1), mat(2, 2);
    return rotationMatrix;
}

Matrix4d RigidTrans::getTransMat() {
    return _transMat;
}

MatrixXd RigidTrans::getIdealProjection() {
    return _idealProj;
}

const MatrixXd RigidTrans::getRotVect() {
    return _rotVect;
}

const MatrixXd RigidTrans::getTransVect() {
    return _transVect;
}

void RigidTrans::prettyPrint(string prefix) {
    cout << prefix <<
            "   _rotVect:       "   << _rotVect.transpose()     <<
            "   _transVect:     "   << _transVect.transpose()   << endl;
}

vector<RigidTrans> RigidTrans::generateRandomRTs(
    double minX, double minY, double minZ
    , double maxX, double maxY, double maxZ
    , size_t nRT) {
    vector<RigidTrans> rts;
    for(int i = 0; i < nRT; i++) {
        double tX = randZeroToOne() * (maxX - minX) + minX;
        double tY = randZeroToOne() * (maxY - minY) + minY;
        double tZ = randZeroToOne() * (maxZ - minZ) + minZ;
        double rX = randZeroToOne();
        double rY = randZeroToOne();
        double rZ = randZeroToOne();
        double rTheta = randZeroToOne() * 2.0 * M_PI;
        rts.push_back(RigidTrans(tX, tY, tZ, rX, rY, rZ, rTheta));
    }
    return rts;
}
