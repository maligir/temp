#ifndef MODEL_CHESSBOARD_H
#define MODEL_CHESSBOARD_H

#include <Eigen/Dense>

class ModelChessboard {
public:
    ModelChessboard(int rows, int cols, double spacing);
    ~ModelChessboard();

    Eigen::MatrixXd getModelCBC2D();
    Eigen::MatrixXd getModelCBH2D();
    Eigen::MatrixXd getModelCBC3D();
    Eigen::MatrixXd getModelCBH3D();

    static Eigen::MatrixXd createModelChessboard(
        int rows, int cols, double spacing);

protected:
    Eigen::MatrixXd _modelCBC2D, _modelCBH2D, _modelCBC3D, _modelCBH3D;
    
};

#endif
