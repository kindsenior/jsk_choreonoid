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

#include "PreviewControlPlugin.h"

namespace cnoid {

class PreviewControlPlugin;

class PreviewControlSetupDialog : public Dialog
{
public:
    ComboParamWidget* modeCombo;

    PreviewControlSetupDialog();
};

class PreviewControlBar : public SetupToolBar
{
public:
    PreviewControlSetupDialog* dialog;

    PreviewControlBar(PreviewControlPlugin* plugin);

    void initialize(PreviewControlPlugin* plugin);
};

}
