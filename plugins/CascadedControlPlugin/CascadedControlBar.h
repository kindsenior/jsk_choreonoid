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
#include <MultiContactStabilizerPlugin/MultiContactStabilizerBar.h>

#include "CascadedControlPlugin.h"

namespace cnoid {

class CascadedControlPlugin;

class CascadedControlSetupLayout : public ParamSetupLayout
{
friend class ParamSetupLayout;

public:
    CascadedControlSetupLayout();
};

class CascadedControlSetupDialog : public ParamSetupDialog
{
public:
    CascadedControlSetupDialog();
    CascadedControlSetupLayout* layout(){return (CascadedControlSetupLayout*) layout_;}
};

class CascadedControlBar : public SetupToolBar
{
public:
    CascadedControlSetupDialog* dialog;

    CascadedControlBar(CascadedControlPlugin* plugin);

    void initialize(CascadedControlPlugin* plugin);
};

}
