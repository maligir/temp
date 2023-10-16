#ifndef HOMOGRAPHY_COST_FUNCTOR_H
#define HOMOGRAPHY_COST_FUNCTOR_H

#include "LinearAlgebraShortcuts.h"
#include "HomCartShortcuts.h"

#include <Eigen/Dense>

using namespace Eigen;

struct HomographyCostFunctor {
public:
    HomographyCostFunctor(MatrixXd modelCBH2D, MatrixXd imageCB)
        : _modelCBH2D(modelCBH2D), _imageCB(imageCB)
        {}

    bool operator()(double const* const* parameters, double* residuals) const {
        MatrixXd h(3,3);
        size_t ctr = 0;
        for(size_t row = 0; row < 3; row++)
            for(size_t col = 0; col < 3; col++)
                h(row,col) = parameters[0][ctr++];

        MatrixXd testCB = divideByLastRowRemoveLastRow(h * _modelCBH2D);
        testCB -= _imageCB;
        MatrixXd vals = columnNorms(testCB);

        for(size_t col = 0; col < testCB.cols(); col++)
            residuals[col] = vals(0,col);
        return true;
    }

protected:
    MatrixXd _modelCBH2D, _imageCB;
};

#endif
