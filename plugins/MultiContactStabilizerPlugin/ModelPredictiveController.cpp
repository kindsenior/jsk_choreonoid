/**
   @author Kunio Kojima
*/

#include "ModelPredictiveController.h"

using namespace hrp;
using namespace std;

// template <class ControllerClass, class ParamClass>
// ModelPredictiveController<ControllerClass,ParamClass>::ModelPredictiveController()
// {
//     isInitial = true;
//     parent = NULL;
// }

// template <class ControllerClass, class ParamClass>
// ParamClass* ModelPredictiveController<ControllerClass,ParamClass>::copyMpcParam(ControllerClass* controller, ParamClass* fromMpcParam)
// {
//     ParamClass* toMpcParam = new ControllerClass(controller, fromMpcParam);
//     return toMpcParam;
// }

// template <class ControllerClass, class ParamClass>
// void ModelPredictiveController<ControllerClass,ParamClass>::calcPhiMatrix()
// {
//     phiMat = dmatrix(stateDim*numWindows_,stateDim);
//     dmatrix lastMat = dmatrix::Identity(stateDim,stateDim);
//     for(typename std::deque<ParamClass*>::iterator iter = mpcParamDeque.begin(); iter != mpcParamDeque.end(); ++iter){
//         int idx = std::distance(mpcParamDeque.begin(), iter);
//         lastMat = (*iter)->systemMat * lastMat;
//         phiMat.block(stateDim*idx,0, stateDim,stateDim) = lastMat;
//     }
// }

// template <class ControllerClass, class ParamClass>
// void ModelPredictiveController<ControllerClass,ParamClass>::calcPsiMatrix()
// {
//     psiMat = dmatrix::Zero(stateDim*numWindows_,psiCols);
//     int colIdx = 0;
//     for(typename std::deque<ParamClass*>::iterator Biter = mpcParamDeque.begin(); Biter != mpcParamDeque.end(); ){// Biterは内側のforループでインクリメント
//         dmatrix lastMat = (*Biter)->inputMat;
//         int cols = lastMat.cols();
//         int Bidx = std::distance(mpcParamDeque.begin(), Biter);// column index
//         psiMat.block(stateDim*Bidx,colIdx, stateDim,cols) = lastMat;
//         for(typename std::deque<ParamClass*>::iterator Aiter = ++Biter; Aiter != mpcParamDeque.end(); ++Aiter){
//             int Aidx = std::distance(mpcParamDeque.begin(), Aiter);// row index
//             lastMat = (*Aiter)->systemMat * lastMat;
//             psiMat.block(stateDim*Aidx,colIdx, stateDim,cols) = lastMat;
//         }
//         colIdx += cols;
//     }
// }

// template <class ControllerClass, class ParamClass>
// void ModelPredictiveController<ControllerClass,ParamClass>::calcEqualConstraints()
// {
//     equalMat = dmatrix::Zero(equalMatRows,URows);
//     equalVec = dvector(equalMatRows);
//     int rowIdx = 0;
//     int colIdx = 0;
//     hrp::dumpMatrix(equalMat, &ParamClass::equalMat, mpcParamDeque, rowIdx, colIdx, blockFlagVec);
//     hrp::dumpVector(equalVec, &ParamClass::equalVec, mpcParamDeque, rowIdx, blockFlagVec);
// }

// template <class ControllerClass, class ParamClass>
// void ModelPredictiveController<ControllerClass,ParamClass>::calcInequalConstraints()
// {
//     inequalMat = dmatrix::Zero(inequalMatRows,URows);
//     inequalMinVec = dvector(inequalMatRows);
//     inequalMaxVec = dvector(inequalMatRows);
//     int rowIdx = 0;
//     int colIdx = 0;
//     hrp::dumpMatrix(inequalMat, &ParamClass::inequalMat, mpcParamDeque, rowIdx, colIdx, blockFlagVec);
//     hrp::dumpVector(inequalMinVec, &ParamClass::inequalMinVec, mpcParamDeque, rowIdx, blockFlagVec);
//     hrp::dumpVector(inequalMaxVec, &ParamClass::inequalMaxVec, mpcParamDeque, rowIdx, blockFlagVec);
// }

// template <class ControllerClass, class ParamClass>
// void ModelPredictiveController<ControllerClass,ParamClass>::calcBoundVectors()
// {
//     minVec = dvector(URows);
//     maxVec = dvector(URows);
//     int rowIdx = 0;
//     hrp::dumpVector(minVec, &ParamClass::minVec, mpcParamDeque, rowIdx, blockFlagVec);
//     hrp::dumpVector(maxVec, &ParamClass::maxVec, mpcParamDeque, rowIdx, blockFlagVec);
// }

// template <class ControllerClass, class ParamClass>
// void ModelPredictiveController<ControllerClass,ParamClass>::calcRefXVector()
// {
//     refXVec = dvector(stateDim*numWindows_);
//     for(typename std::deque<ParamClass*>::iterator iter = mpcParamDeque.begin(); iter != mpcParamDeque.end(); ++iter){
//         int idx = std::distance(mpcParamDeque.begin(), iter);
//         refXVec.block(stateDim*idx,0, stateDim,1) = (*iter)->refStateVec;
//     }
// }

// template <class ControllerClass, class ParamClass>
// void ModelPredictiveController<ControllerClass,ParamClass>::calcErrorWeightMatrix()
// {
//     errorWeightMat = dmatrix::Zero(stateDim*numWindows_,stateDim*numWindows_);
//     for(typename std::deque<ParamClass*>::iterator iter = mpcParamDeque.begin(); iter != mpcParamDeque.end(); ++iter){
//         int idx = std::distance(mpcParamDeque.begin(), iter);
//         errorWeightMat.block(stateDim*idx,stateDim*idx, stateDim,stateDim) = (*iter)->errorWeightVec.asDiagonal();
//     }
// }

// template <class ControllerClass, class ParamClass>
// void ModelPredictiveController<ControllerClass,ParamClass>::calcInputWeightMatrix()
// {
//     inputWeightMat = dmatrix::Zero(psiCols,psiCols);
//     int rowIdx = 0;
//     for(typename std::deque<ParamClass*>::iterator iter = mpcParamDeque.begin(); iter != mpcParamDeque.end(); ++iter){
//          int rows = (*iter)->inputWeightVec.rows();
//          inputWeightMat.block(rowIdx,rowIdx, rows,rows) = (*iter)->inputWeightVec.asDiagonal();
//         rowIdx += rows;
//     }
// }

// template <class ControllerClass, class ParamClass>
// void ModelPredictiveController<ControllerClass,ParamClass>::calcBlockMatrix()
// {
//     blockMat = dmatrix::Zero(psiCols,URows);
//     blockMatInv = dmatrix::Zero(URows,psiCols);
//     std::vector<bool>::iterator blockFlagIter = blockFlagVec.begin();
//     typename std::deque<ParamClass*>::iterator colIter = mpcParamDeque.begin();
//     int count = 1, rowIdx = 0, colIdx = 0;
//     for(typename std::deque<ParamClass*>::iterator rowIter = mpcParamDeque.begin(); rowIter != mpcParamDeque.end(); ++rowIter, ++colIter){
//         int rows,cols;
//         rows = (*rowIter)->inputDim;
//         if(*blockFlagIter){
//             cols = (*colIter)->inputDim;
//             blockMatInv.block(colIdx,rowIdx, cols,rows) = dmatrix::Identity(cols,rows);// blockMatInv
//         }
//         blockMat.block(rowIdx,colIdx, rows,cols) = dmatrix::Identity(rows,cols);// blockMat
//         if(*(++blockFlagIter)){
//             colIdx += cols;
//         }
//         rowIdx += rows;
//     }
// }

// template <class ControllerClass, class ParamClass>
// void ModelPredictiveController<ControllerClass,ParamClass>::updateX0Vector()
// {
//     ParamClass* mpcParam = mpcParamDeque[0];
//     // U->u0
//     // x0 = A0*x0 + B'0*u0
//     dmatrix x1 = mpcParam->systemMat*x0 + mpcParam->inputMat*U.block(0,0, mpcParam->inputMat.cols(),1);


//     if(parent != NULL){
//         ControllerClass* root = rootController();
//         int stepNum = parent->dt/root->dt;// root->dtで割り切れる?
//         // int nextPushIndex;
//         // if(!parent->preMpcParamDeque.empty()) nextPushIndex = parent->preMpcParamDeque.back()->index() + stepNum;
//         // else if(!parent->mpcParamDeque.empty()) nextPushIndex = parent->mpcParamDeque.back()->index() + stepNum;
//         // else nextPushIndex = 0;

//         int x0Index = mpcParamDeque[0]->index();
//         int x1Index = mpcParamDeque[1]->index();// 0除算の可能性 最後以降は補間しないから大丈夫?
//         cout << x0Index << ": " << x0.transpose() << endl;
//         cout << x1Index << ": " << x1.transpose() << endl;
//         cout << "root: " << root->dt << "  parent:" << parent->dt << endl;
//         typename std::deque<ParamClass*>::iterator rootPreDequeIter = root->preMpcParamDeque.begin();
//         ParamClass* targetMpcParam;
//         while((*rootPreDequeIter)->index() < x0Index) rootPreDequeIter += stepNum;
//         while((*rootPreDequeIter)->index() < x1Index){
//             if(root == parent){
//                 targetMpcParam = *rootPreDequeIter;
//             }else{
//                 parent->preMpcParamDeque.push_back(copyMpcParam(parent, *rootPreDequeIter));// pickup mpcParam from root mpc dtが変わるので行列は生成し直す
//                 targetMpcParam = parent->preMpcParamDeque.back();
//             }
//             targetMpcParam->setRefStateVector(x0 + (x1-x0)*((*rootPreDequeIter)->index() - x0Index)/(double)(x1Index - x0Index)); // interpolate refX,U
//             rootPreDequeIter += stepNum;
//         }
//     }

//     x0 = x1;
// }

// template <class ControllerClass, class ParamClass>
// void ModelPredictiveController<ControllerClass,ParamClass>::calcAugmentedMatrix()
// {
//     psiCols = 0;
//     equalMatRows = 0;
//     inequalMatRows = 0;
//     URows = 0;
//     std::vector<bool>::iterator blockFlagIter = blockFlagVec.begin();
//     for(typename std::deque<ParamClass*>::iterator iter = mpcParamDeque.begin(); iter != mpcParamDeque.end(); ++iter, ++blockFlagIter){
//         psiCols += (*iter)->inputMat.cols();
//         if(*blockFlagIter){
//             URows += (*iter)->inputDim;
//             equalMatRows += (*iter)->equalMat.rows();
//             inequalMatRows += (*iter)->inequalMat.rows();
//         }
//     }

//     if(isInitial){
//         x0 = dvector(stateDim);
//         x0 = mpcParamDeque[0]->refStateVec;
//         isInitial = false;
//     }

//     calcPhiMatrix();
//     calcPsiMatrix();
//     calcEqualConstraints();
//     calcInequalConstraints();
//     calcBoundVectors();
//     calcRefXVector();
//     calcErrorWeightMatrix();
//     calcInputWeightMatrix();
//     calcBlockMatrix();
//     U = dvector::Zero(URows);

// }

// template <class ControllerClass, class ParamClass>
// ControllerClass* ModelPredictiveController<ControllerClass,ParamClass>::rootController()
// {
//     if(parent != NULL) return parent->rootController();
//     return this;
// }

// template <class ControllerClass, class ParamClass>
// void ModelPredictiveController<ControllerClass,ParamClass>::pushAllPreMPCParamFromRoot()
// {
//     ControllerClass* root = rootController();
//     int stepNum = dt/root->dt;// root->dtで割り切れる?
//     typename std::deque<ParamClass*>::iterator iter = root->preMpcParamDeque.begin();
//     while(iter != root->preMpcParamDeque.end()){
//         preMpcParamDeque.push_back(copyMpcParam(this ,*iter));
//         for(int i=0; i<stepNum && iter !=root->preMpcParamDeque.end(); ++i, ++iter);
//     }
// }
