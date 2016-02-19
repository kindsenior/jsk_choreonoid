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

#include <UtilPlugin/ParamSetupDialog.h>

#include "MultiContactStabilizerPlugin.h"

namespace cnoid {

class MultiContactStabilizerPlugin;

class MultiContactStabilizerSetupDialog : public ParamSetupDialog
{
public:
    QVBoxLayout* vbox;

    SpinParamWidget* errorCMWeightSpin;
    SpinParamWidget* errorMomentumWeightSpin;
    SpinParamWidget* errorAngularMomentumWeightSpin;
    SpinParamWidget* inputForceWeightSpin;
    SpinParamWidget* inputMomentWeightSpin;
    SpinParamWidget* numWindowsSpin;

    CheckBox saveParameterInFileNameCheck;

    MultiContactStabilizerSetupDialog();

    void storeState(Archive& archive);
    void restoreState(const Archive& archive);
};


class MultiContactStabilizerBar : public ToolBar
{
private:
    bool storeState(Archive& archive);
    bool restoreState(const Archive& archive);

public:
    MultiContactStabilizerSetupDialog* dialog;

    MultiContactStabilizerBar(MultiContactStabilizerPlugin* plugin);
};

}
