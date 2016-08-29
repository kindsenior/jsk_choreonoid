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

#include "SlideFrictionControlPlugin.h"

namespace cnoid {

class SlideFrictionControlPlugin;

class SlideFrictionControlSetupLayout : public ParamSetupLayout
{
public:
    SpinParamWidget* dtSpin;
    SpinParamWidget* errorCMWeightSpin;
    SpinParamWidget* errorMomentumWeightSpin;
    SpinParamWidget* errorAngularMomentumWeightSpin;
    SpinParamWidget* errorYawAngularMomentumWeightSpin;
    SpinParamWidget* inputForceWeightSpin;
    SpinParamWidget* inputMomentWeightSpin;
    SpinParamWidget* inputYawMomentWeightSpin;
    SpinVectorParamWidget* blockSpinVec;

    SlideFrictionControlSetupLayout();
};

class SlideFrictionControlSetupDialog : public ParamSetupDialog
{
public:
    SlideFrictionControlSetupDialog();
    SlideFrictionControlSetupLayout* layout(){return (SlideFrictionControlSetupLayout*) layout_;}
};

class SlideFrictionControlBar : public SetupToolBar
{
public:
    SlideFrictionControlSetupDialog* dialog;

    SlideFrictionControlBar(SlideFrictionControlPlugin* plugin);

    void initialize(SlideFrictionControlPlugin* plugin);
};

}
