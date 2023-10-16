#ifndef HOM_CART_SHORTCUTS_H
#define HOM_CART_SHORTCUTS_H

#include <Eigen/Dense>

Eigen::MatrixXd addARowOfConst(Eigen::MatrixXd inM, double val);

Eigen::MatrixXd divideByLastRowRemoveLastRow(Eigen::MatrixXd inM);

#endif
