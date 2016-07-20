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

#include "RMControlPlugin.h"

namespace cnoid {

enum InitialTrajectoryMode : int { Generate = 0, Load = 1 };

class RMControlPlugin;

class RMControlSetupDialog : public ParamSetupDialog
{
public:
    ComboParamWidget* initialTrajectoryCombo;
    RMControlSetupDialog();
};


class RMControlBar : public SetupToolBar
{
public:
    RMControlSetupDialog* dialog;

    RMControlBar(RMControlPlugin* plugin);

    void initialize(RMControlPlugin* plugin);
};

}
