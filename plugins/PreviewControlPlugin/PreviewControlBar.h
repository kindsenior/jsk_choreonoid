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

#include "PreviewControlPlugin.h"

namespace cnoid {

class PreviewControlPlugin;

class PreviewControlSetupDialog : public Dialog
{
public:
    PreviewControlSetupDialog();
};

class PreviewControlBar : public ToolBar
{
protected:

public:

    PreviewControlSetupDialog* dialog;

    PreviewControlBar(PreviewControlPlugin* plugin);
};

}
