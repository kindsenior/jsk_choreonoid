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

#include "MultiContactStabilizerPlugin.h"

namespace cnoid {

class MultiContactStabilizerSetupDialog;
class MultiContactStabilizerPlugin;

class MultiContactStabilizerBar : public ToolBar
{
private:
    MultiContactStabilizerSetupDialog* dialog;
    bool storeState(Archive& archive);
    bool restoreState(const Archive& archive);

public:
    MultiContactStabilizerBar(MultiContactStabilizerPlugin* plugin);
};

}
