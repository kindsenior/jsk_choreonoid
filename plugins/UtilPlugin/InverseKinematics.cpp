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
Eigen::MatrixXd cnoid::SRInverse(const Eigen::MatrixXd& m, const double weight, const double manipulability_thresh)
{
    Eigen::MatrixXd w;
    return SRInverse(m, w, weight, manipulability_thresh);
}

Eigen::MatrixXd cnoid::SRInverse(const Eigen::MatrixXd& m, Eigen::MatrixXd& weight_mat, const double weight, const double manipulability_thresh)
{
    const int numCols = m.cols();
    if ( weight_mat.cols() != numCols || weight_mat.rows() != numCols ) {
        weight_mat = Eigen::MatrixXd::Identity(numCols, numCols);
    }

    double manipulability = sqrt((m*m.transpose()).determinant());
    return weight_mat*m.transpose() * ((m*weight_mat*m.transpose() + (manipulability < manipulability_thresh ? weight*pow(1-manipulability/manipulability_thresh,2) : 0) * Eigen::MatrixXd::Identity(m.rows(),m.rows())).inverse());
}

// Eigen::MatrixXd cnoid::calcInverseJacobian(const Eigen::MatrixXd& m)
Eigen::MatrixXd cnoid::inverseJacobian(JointPathPtr& jp)
{
    MatrixXd J;
    jp->calcJacobian(J);
    // const int numJoints = jp->numJoints();
    // Eigen::MatrixXd weight_mat = Eigen::MatrixXd::Identity(numJoints, numJoints);

    // return PseudoInverse(m);
    // return SRInverse(J, weight_mat);
    // return SRInverse(J);
    return J.inverse();
}

MatrixXd cnoid::threshMatrix(const MatrixXd& m, double thresh) {
    MatrixXd ret;
    int numRows = m.rows(), numCols = m.cols();
    ret.resize(numRows,numCols);
    for(int i=0; i<numRows; ++i){
        for(int j=0; j<numCols; ++j) {
            ret(i,j) = fabs(m(i,j)) > thresh ? m(i,j) : 0;
        }
    }
    return ret;
}

MatrixXd cnoid::extractMatrixColumn(const MatrixXd& m, const std::set<int>& jointIndexSet) {
    double numJoints = jointIndexSet.size();
    MatrixXd ret;
    ret.resize(m.rows(),numJoints);
    std::set<int>::iterator iter = jointIndexSet.begin();
    for(int i=0; i<numJoints; ++i,++iter) ret.col(i) = m.col(*iter);
    return ret;
}

MatrixXd cnoid::extractMatrixRow(const MatrixXd& m, const std::set<int>& rowIndexSet) {
    double numRows = rowIndexSet.size();
    MatrixXd ret;
    ret.resize(numRows,m.cols());
    std::set<int>::iterator iter = rowIndexSet.begin();
    for(int i=0; i<numRows; ++i,++iter) ret.row(i) = m.row(*iter);
    return ret;
}
