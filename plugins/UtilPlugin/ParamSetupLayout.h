/**
   @author Kunio Kojima
*/
#pragma once

#include "ParamWidget.h"

namespace cnoid {

class ParamSetupLayout : public QVBoxLayout
{
protected:
    ParamNode* paramNodes;
    CheckBox saveParameterInFileNameCheck_;

    QHBoxLayout* newRow()
    {
        QHBoxLayout* hbox = new QHBoxLayout();
        hbox->setSpacing(2);
        hbox->setContentsMargins(2, 2, 2, 2);
        this->addLayout(hbox);
        return hbox;
    }

public:
    ParamSetupLayout()
    {
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
