#ifndef RIGID_TRANS_H
#define RIGID_TRANS_H

#include <Eigen/Dense>

#include <string>
#include <vector>

class RigidTrans {
public:
    RigidTrans(Eigen::MatrixXd rotVect, Eigen::MatrixXd transVect);
    RigidTrans(
        double tX, double tY, double tZ
        , double rX, double rY, double rZ, double rTheta);
    RigidTrans(
        double tX, double tY, double tZ
        , double rX, double rY, double rZ);

    Eigen::Quaternion<double> getRotationQuat();
    Eigen::Matrix3d getRotationMat();
    Eigen::Matrix4d getTransMat();
    Eigen::MatrixXd getIdealProjection();

    const Eigen::MatrixXd getRotVect();
    const Eigen::MatrixXd getTransVect();

    void prettyPrint(std::string prefix);

    static std::vector<RigidTrans> generateRandomRTs(
        double minX, double minY, double minZ
        , double maxX, double maxY, double maxZ
        , size_t nRT);

protected:
    void init(
        double tX, double tY, double tZ
        , double rX, double rY, double rZ);
    void computeTransMatAndIdealProj();

    //double _rTheta;
    Eigen::MatrixXd _rotVect, _transVect, _idealProj;
    Eigen::Matrix4d _transMat;
};

#endif
