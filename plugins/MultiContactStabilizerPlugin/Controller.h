#include <iostream>
#include <sstream>
#include <fstream>
#include <map>

#include <boost/bind.hpp>

#include <qpOASES.hpp>

#include <cnoid/Plugin>
#include <cnoid/ItemTreeView>
#include <cnoid/BodyItem>
#include <cnoid/MessageView>

#include <cnoid/src/PoseSeqPlugin/PoseSeqItem.h>
#include <cnoid/src/PoseSeqPlugin/BodyMotionGenerationBar.h>
#include <cnoid/FileUtil>

#include <UtilPlugin/UtilPlugin.h>
#include "typedef.h"

namespace hrp {

typedef struct
{
    int contactState;
    Vector3 p;
    Matrix33 R;
    std::vector<Vector3> edgeVec;
    double mu;
}ContactConstraintParam;

typedef struct
{
    int stateDim;
    int inputDim;
    dmatrix systemMat;// A
    dmatrix inputMat;// B
    // dmatrix outputMat;
}ModelPreviewControllerParam;

class ModelPreviewControl
{
public:
    int numWindows;// N
    std::vector<MPCParam> MPCParamVec;
    dvector x0;
    dmatrix psiMat;
    dmatrix phiMat;
};

typedef struct
{
    ContactConstraintParam contactStateParam;
    Vector3 CM;
    Vector3 P;
    Vector3 L;
}MultiContactStabilizerParam;

class MultiContactStabilizer : public ModelPreviewControl
{
public:
    std::vector<MCSParam> MCSParamVec;
};

}
