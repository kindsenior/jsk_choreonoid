/**
   @author Kunio Kojima
*/
#pragma once

#include "ParamWidget.h"

namespace cnoid {

class ParamSetupLayout : public QVBoxLayout
{
protected:
    QBoxLayout* vbox_;
    ParamNode* paramNodes;
    CheckBox saveParameterInFileNameCheck_;

    QHBoxLayout* newRow(QVBoxLayout* vbox)
    {
        QHBoxLayout* hbox = new QHBoxLayout();
        hbox->setSpacing(2);
        hbox->setContentsMargins(2, 2, 2, 2);
        vbox->addLayout(hbox);
        return hbox;
    }

public:
    ParamSetupLayout(QVBoxLayout* vbox)
    {
        vbox_ = vbox;
        paramNodes = NULL;
    }

    virtual void storeState(Archive& archive)
    {
        if(paramNodes != NULL) paramNodes->storeState(archive);
    }
    virtual void restoreState(const Archive& archive)
    {
        if(paramNodes != NULL) paramNodes->restoreState(archive);
    }

    virtual std::string getParamString()
    {
        std::stringstream ss;
        if(saveParameterInFileNameCheck_.isChecked() && paramNodes != NULL) ss << paramNodes->getParamString();
        return ss.str();
    }
};

}
