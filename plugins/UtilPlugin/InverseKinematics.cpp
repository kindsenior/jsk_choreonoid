/**
   @author Kunio Kojima
*/

#include "InverseKinematics.h"

using namespace cnoid;

template <typename t_matrix>
t_matrix cnoid::PseudoInverse(const t_matrix& m, const double &tolerance=1.e-6)
{
    using namespace Eigen;
    typedef JacobiSVD<t_matrix> TSVD;
    unsigned int svd_opt(ComputeThinU | ComputeThinV);
    if(m.RowsAtCompileTime!=Dynamic || m.ColsAtCompileTime!=Dynamic)
        svd_opt= ComputeFullU | ComputeFullV;
    TSVD svd(m, svd_opt);
    const typename TSVD::SingularValuesType &sigma(svd.singularValues());
    typename TSVD::SingularValuesType sigma_inv(sigma.size());
    for(long i=0; i<sigma.size(); ++i){
        if(sigma(i) > tolerance)
            sigma_inv(i)= 1.0/sigma(i);
        else
            sigma_inv(i)= 0.0;
    }
    return svd.matrixV()*sigma_inv.asDiagonal()*svd.matrixU().transpose();
}

template Eigen::MatrixXd cnoid::PseudoInverse(const Eigen::MatrixXd&, const double&);

// cannot call default argument by reference so have to overload function
Eigen::MatrixXd cnoid::SRInverse(const Eigen::MatrixXd& m, const double weight, const double manipulability_const)
{
    Eigen::MatrixXd w;
    return SRInverse(m, w, weight, manipulability_const);
}

Eigen::MatrixXd cnoid::SRInverse(const Eigen::MatrixXd& m, Eigen::MatrixXd& weight_mat, const double weight, const double manipulability_const)
{
    const int numCols = m.cols();
    if ( weight_mat.cols() != numCols || weight_mat.rows() != numCols ) {
        weight_mat = Eigen::MatrixXd::Identity(numCols, numCols);
    }

    double manipulability = sqrt((m*m.transpose()).determinant());
    return weight_mat*m.transpose() * ((m*weight_mat*m.transpose() + weight*exp(-manipulability/manipulability_const) * Eigen::MatrixXd::Identity(m.rows(),m.rows())).inverse());
}

// Eigen::MatrixXd cnoid::calcInverseJacobian(const Eigen::MatrixXd& m)
Eigen::MatrixXd cnoid::inverseJacobian(JointPathPtr& jp)
{
    MatrixXd J;
    jp->calcJacobian(J);
    const int numJoints = jp->numJoints();
    Eigen::MatrixXd weight_mat = Eigen::MatrixXd::Identity(numJoints, numJoints);

    return SRInverse(J, weight_mat);
}
