#include <vision_geometry/LinearAlgebraShortcuts.h>

using namespace cv;
using namespace Eigen;
using namespace std;

MatrixXd rightNullSpace(MatrixXd A) {
    JacobiSVD<MatrixXd> svd(A, Eigen::ComputeFullV);
    return svd.matrixV().block(
        0, svd.matrixV().cols() - 1, svd.matrixV().rows(), 1);
}

Mat convertToCVMat(MatrixXd A) {
    Mat cvmat(A.rows(), A.cols(), CV_64F);
    for(size_t row = 0; row < A.rows(); row++)
        for(size_t col = 0; col < A.cols(); col++)
            cvmat.at<double>(row, col) = A(row, col);
    return cvmat;
}

MatrixXd convertToEigenMatF(Mat A) {
    //cout << "convertToEigenMat: " << A.rows << "   " << A.cols << endl;
    MatrixXd eigenMat(A.rows, A.cols);
    for(size_t row = 0; row < eigenMat.rows(); row++)
        for(size_t col = 0; col < eigenMat.cols(); col++)
            eigenMat(row, col) = A.at<float>(row, col);
    return eigenMat;
}

MatrixXd convertToEigenMatD(Mat A) {
    //cout << "convertToEigenMat: " << A.rows << "   " << A.cols << endl;
    MatrixXd eigenMat(A.rows, A.cols);
    for(size_t row = 0; row < eigenMat.rows(); row++)
        for(size_t col = 0; col < eigenMat.cols(); col++)
            eigenMat(row, col) = A.at<double>(row, col);
    return eigenMat;
}

Eigen::MatrixXd convertToEigenMat(std::vector<std::vector<cv::Point2f> > c) {
    MatrixXd eigenMat(2, c.size() * c[0].size());
    size_t ctr = 0;
    for(size_t i = 0; i < c.size(); i++) {
        for(size_t j = 0; j < c[i].size(); j++) {
            eigenMat(0, ctr++) = c[i][j].x;
            eigenMat(1, ctr++) = c[i][j].y;
        }
    }
    return eigenMat;
}

vector<MatrixXd> convertToEigenMatF(vector<Mat> A) {
    vector<MatrixXd> retVal;
    for(size_t i = 0; i < A.size(); i++) {
        MatrixXd B = convertToEigenMatF(A[i]);
        //cout << "convertToEigenMat: B" << endl;
        //cout << B << endl;
        retVal.push_back(B);
    }
    return retVal;
}

vector<MatrixXd> convertToEigenMatD(vector<Mat> A) {
    vector<MatrixXd> retVal;
    for(size_t i = 0; i < A.size(); i++) {
        MatrixXd B = convertToEigenMatD(A[i]);
        //cout << "convertToEigenMat: B" << endl;
        //cout << B << endl;
        retVal.push_back(B);
    }
    return retVal;
}

MatrixXd convert2ChanCVMatToEigenMatF(Mat A) {
    //cout << "convertToEigenMat: " << A.rows << "   " << A.cols << endl;
    //Mat B[2];
    //split(A, B);
    cv::Point2f *B = A.ptr<cv::Point2f>(); // TODO verify
    MatrixXd eigenMat(2, A.rows);
    for(size_t row = 0; row < A.rows; row++) {
        eigenMat(0, row) = B[row].x;
        eigenMat(1, row) = B[row].y;
    }
    return eigenMat;
}

vector<MatrixXd> convert2ChanCVMatToEigenMatF(vector<Mat> A) {
    vector<MatrixXd> retVal;
    for(size_t i = 0; i < A.size(); i++) {
        MatrixXd B = convert2ChanCVMatToEigenMatF(A[i]);
        //cout << "convertToEigenMat: B" << endl;
        //cout << B << endl;
        retVal.push_back(B);
    }
    return retVal;
}

MatrixXd columnNorms(MatrixXd A) {
    MatrixXd retVal(1, A.cols());
    for(size_t col = 0; col < A.cols(); col++) {
        retVal(0,col) = 0.0;
        for(size_t row = 0; row < A.rows(); row++) {
            retVal(0,col) += A(row, col) * A(row, col);
        }
        retVal(0,col) = sqrt(retVal(0,col));
    }
    return retVal;
}
