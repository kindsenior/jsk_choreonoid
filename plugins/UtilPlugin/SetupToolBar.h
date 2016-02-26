/**
   @author Kunio Kojima
*/
#pragma once

#include "ParamSetupDialog.h"

namespace cnoid {

class SetupToolBar : public ToolBar
{
public:
    ParamSetupDialog* dialog;

    SetupToolBar(QString string) : ToolBar(string){};

    virtual void initialize(ParamSetupDialog* dialog_)
    {
        dialog = dialog_;
    };

    virtual bool storeState(Archive& archive)
    {
        dialog->storeState(archive);
        return true;
    }

    virtual bool restoreState(const Archive& archive)
    {
        dialog->restoreState(archive);
        return true;
    }

};

}
