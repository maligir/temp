#include <vision_geometry/ModelChessboard.h>
#include <vision_geometry/HomCartShortcuts.h>

using namespace Eigen;

ModelChessboard::ModelChessboard(int rows, int cols, double spacing) :
    _modelCBC2D(createModelChessboard(rows, cols, spacing))
    , _modelCBH2D(addARowOfConst(_modelCBC2D, 1.0))
    , _modelCBC3D(addARowOfConst(_modelCBC2D, 0.0))
    , _modelCBH3D(addARowOfConst(_modelCBC3D, 1.0)) {}

ModelChessboard::~ModelChessboard() {}

MatrixXd ModelChessboard::getModelCBC2D()   {   return _modelCBC2D; }
MatrixXd ModelChessboard::getModelCBH2D()   {   return _modelCBH2D; }
MatrixXd ModelChessboard::getModelCBC3D()   {   return _modelCBC3D; }
MatrixXd ModelChessboard::getModelCBH3D()   {   return _modelCBH3D; }

MatrixXd ModelChessboard::createModelChessboard(
    int rows, int cols, double spacing) {
    double xOff = (double)cols * spacing / 2.0;
    double yOff = (double)rows * spacing / 2.0;
    MatrixXd m(2, rows * cols);
    for(int row = 0; row < rows; row++) {
        for(int col = 0; col < cols; col++) {
            m(0, row * cols + col) = ((double)col) * spacing - xOff;
            m(1, row * cols + col) = ((double)row) * spacing - yOff;
        }
    }

    return m;
}
