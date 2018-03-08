/**
   @author Kunio Kojima
*/
#pragma once

#include <iostream>
#include <set>

#include <cnoid/src/Util/EigenTypes.h>
#include <cnoid/JointPath>
#include <cnoid/Body>

namespace cnoid {

template <typename t_matrix>
t_matrix PseudoInverse(const t_matrix& m, const double &tolerance=1.e-6);

Eigen::MatrixXd SRInverse(const Eigen::MatrixXd& m, const double weight = 1, const double manipulability_const = 0.01);
Eigen::MatrixXd SRInverse(const Eigen::MatrixXd& m, Eigen::MatrixXd& weight_mat, const double weight = 1, const double manipulability_const = 0.01);

Eigen::MatrixXd inverseJacobian(JointPathPtr& jp);

MatrixXd extractMatrix(const MatrixXd& m, const std::set<int>& jointIdSet);

class ConstraintJointPath : public JointPath
{
public:
    ConstraintJointPath(Link* baseLink, Link* targetLink) : JointPath(baseLink, targetLink) {
        numVirtualBaseJoints_ = 6;

        numBodyJoints_ = baseLink->body()->numJoints();
        numTotalJoints_ = numBodyJoints_ + numVirtualBaseJoints_;

        jacobianWholeBody_ = MatrixXd::Zero(6,numTotalJoints_);

        exclusiveJointIdSet_.clear();
        for(int i=0; i<numJoints(); ++i) exclusiveJointIdSet_.insert(this->joint(i)->jointId());

        commonJointPathPtr = std::make_shared<JointPath>(baseLink->body()->rootLink(), baseLink);
        std::cout << "commonNumJoints:" << commonJointPathPtr->numJoints();
        for(int i=0; i<commonJointPathPtr->numJoints(); ++i) std::cout << " " << commonJointPathPtr->joint(i)->jointId(); std::cout << std::endl;
        commonJointIdSet_.clear();
        for(int i=0; i<commonJointPathPtr->numJoints(); ++i) commonJointIdSet_.insert(commonJointPathPtr->joint(i)->jointId());
        for(int i=0; i<numVirtualBaseJoints_; ++i) commonJointIdSet_.insert(i+numBodyJoints_);

        twist.resize(6);

        jointExtendMat_ = MatrixXd::Zero(numTotalJoints_, numJoints());
        for(int i=0; i<numJoints(); ++i) jointExtendMat_(this->joint(i)->jointId(),i) = 1;
    }

    void calcWholeBodyJacobian() {
    }

    void update() {
        // jacobianWholeBody;
        MatrixXd J;
        J.resize(6,numJoints()); this->calcJacobian(J);
        for(int i=0; i<this->numJoints(); ++i) jacobianWholeBody_.col(this->joint(i)->jointId()) = J.col(i);

        J.resize(6,commonJointPathPtr->numJoints()); commonJointPathPtr->calcJacobian(J);
        for(int i=0; i<commonJointPathPtr->numJoints(); ++i) jacobianWholeBody_.col(commonJointPathPtr->joint(i)->jointId()) = J.col(i);

        jacobianWholeBody_.block(0,numBodyJoints_, 6,numVirtualBaseJoints_) = MatrixXd::Identity(6,numVirtualBaseJoints_);
        Vector3d vec = this->endLink()->p() - baseLink()->body()->rootLink()->p();
        for(int i=0; i<3; ++i) jacobianWholeBody_.block(0,numBodyJoints_+3+i, 3,1) = Matrix3::Identity().col(i).cross(vec);
    }

    std::set<int>& exclusiveJointIdSet() { return exclusiveJointIdSet_; }
    std::set<int>& commonJointIdSet() { return commonJointIdSet_; }

    MatrixXd& jointExtendMatrix() { return jointExtendMat_; }
    MatrixXd& jacobianWholeBody() { return jacobianWholeBody_; }

    SE3 se3;
    VectorXd twist;

protected:
    int numVirtualBaseJoints_;
    int numBodyJoints_;
    int numTotalJoints_; // include virtual base joints

    JointPathPtr commonJointPathPtr;

    std::set<int> exclusiveJointIdSet_;
    std::set<int> commonJointIdSet_;

    MatrixXd jacobianWholeBody_;
    MatrixXd jointExtendMat_;
};

typedef std::shared_ptr<ConstraintJointPath> ConstraintJointPathPtr;

class WholeBodyConstraint
{
public:
    WholeBodyConstraint(BodyPtr& bodyPtr) {
        body = bodyPtr;

        numVirtualBaseJoints_ = 6;
        numBodyJoints_ = body->numJoints();
        numTotalJoints_ = numBodyJoints_ + numVirtualBaseJoints_;
    }

    void update() {
        exclusiveJointIdSet_.clear();
        for(auto jointPathPtr : constraintJointPathVec) {
            jointPathPtr->update();
            for(int i=0; i<jointPathPtr->numJoints(); ++i) exclusiveJointIdSet_.insert(jointPathPtr->joint(i)->jointId());
        }

        freeJointIdSet_.clear();
        for(int i=0; i<body->numJoints(); ++i)
            if( exclusiveJointIdSet_.find(body->joint(i)->jointId()) == exclusiveJointIdSet_.end() ) freeJointIdSet_.insert(body->joint(i)->jointId());

        complementJointIdSet_ = freeJointIdSet_;
        for(int i=0; i<numVirtualBaseJoints_; ++i) complementJointIdSet_.insert(numBodyJoints_+i);

        jointExtendMat_ = MatrixXd::Zero(numTotalJoints_, freeJointIdSet_.size()+numVirtualBaseJoints_);
        std::set<int>::iterator iter = freeJointIdSet_.begin();
        for(int i=0; i<freeJointIdSet_.size(); ++i,++iter) jointExtendMat_(*iter,i) = 1;
        for(int i=0; i<numVirtualBaseJoints_; ++i) jointExtendMat_(numBodyJoints_+i,freeJointIdSet_.size()+i) = 1;

    }

    std::set<int>& freeJointIdSet() { return freeJointIdSet_; }
    std::set<int>& complementJointIdSet() { return complementJointIdSet_; }
    int numVirtualBaseJoints() { return numVirtualBaseJoints_; }

    MatrixXd& jointExtendMatrix() { return jointExtendMat_; }

    std::vector<ConstraintJointPathPtr> constraintJointPathVec;

protected:
    double numVirtualBaseJoints_;
    double numBodyJoints_;
    double numTotalJoints_;

    BodyPtr body;

    std::set<int> exclusiveJointIdSet_;
    std::set<int> freeJointIdSet_;
    std::set<int> complementJointIdSet_;

    MatrixXd jointExtendMat_;
};

typedef std::shared_ptr<WholeBodyConstraint> WholeBodyConstraintPtr;

}
