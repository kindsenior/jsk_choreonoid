/**
   @author Kunio Kojima
*/
#pragma once

#include <cnoid/ToolBar>
#include <cnoid/Dialog>
#include <cnoid/SpinBox>
// #include <cnoid/Separator>
#include <cnoid/Buttons>
#include <cnoid/CheckBox>
#include <cnoid/LineEdit>
#include <QDialogButtonBox>

#include <cnoid/Archive>

#include <UtilPlugin/SetupToolBar.h>

#include "MultiContactStabilizerPlugin.h"

namespace cnoid {

class MultiContactStabilizerPlugin;

class MultiContactStabilizerSetupLayout : public ParamSetupLayout
{
public:
    SpinParamWidget* errorCMWeightSpin;
    SpinParamWidget* errorMomentumWeightSpin;
    SpinParamWidget* errorAngularMomentumWeightSpin;
    SpinParamWidget* inputForceWeightSpin;
    SpinParamWidget* inputMomentWeightSpin;
    SpinVectorParamWidget* blockSpinVec;

    MultiContactStabilizerSetupLayout(QVBoxLayout* vbox);
};

class MultiContactStabilizerSetupDialog : public ParamSetupDialog
{
public:
    MultiContactStabilizerSetupDialog();
    MultiContactStabilizerSetupLayout* layout(){return (MultiContactStabilizerSetupLayout*) layout_;}
};

class MultiContactStabilizerBar : public SetupToolBar
{
public:
    MultiContactStabilizerSetupDialog* dialog;

    MultiContactStabilizerBar(MultiContactStabilizerPlugin* plugin);

    void initialize(MultiContactStabilizerPlugin* plugin);
};

}
