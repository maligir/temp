#include <vision_geometry/HomCartShortcuts.h>

using namespace Eigen;

MatrixXd addARowOfConst(MatrixXd inM, double val) {
    MatrixXd outM(inM.rows() + 1, inM.cols());
    outM.block(0, 0, inM.rows(), inM.cols()) = inM;
    outM.block(inM.rows(), 0, 1, inM.cols()) =
        MatrixXd::Constant(1, inM.cols(),val);
    return outM;
}

MatrixXd divideByLastRowRemoveLastRow(MatrixXd inM) {
    MatrixXd outM(inM.rows() - 1, inM.cols());
    MatrixXd divM = inM.block(inM.rows() - 1, 0, 1, inM.cols()).cwiseInverse();
    for(int row = 0; row < outM.rows(); row++) {
        outM.block(row, 0, 1, inM.cols()) =
            inM.block(row, 0, 1, inM.cols()).cwiseProduct(divM);
    }
    return outM;
}
