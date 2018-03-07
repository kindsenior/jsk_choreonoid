/**
   @author Kunio Kojima
*/
#pragma once

#include <iostream>

#include <cnoid/src/Util/EigenTypes.h>
#include <cnoid/JointPath>

namespace cnoid {

template <typename t_matrix>
t_matrix PseudoInverse(const t_matrix& m, const double &tolerance=1.e-6);

Eigen::MatrixXd SRInverse(const Eigen::MatrixXd& m, const double weight = 1, const double manipulability_const = 0.01);
Eigen::MatrixXd SRInverse(const Eigen::MatrixXd& m, Eigen::MatrixXd& weight_mat, const double weight = 1, const double manipulability_const = 0.01);

Eigen::MatrixXd inverseJacobian(JointPathPtr& jp);

}
