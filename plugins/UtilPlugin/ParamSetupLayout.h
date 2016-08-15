/**
   @author Kunio Kojima
*/
#pragma once

#include "ParamWidget.h"

namespace cnoid {

class ParamSetupLayout : public QVBoxLayout
{
protected:
    ParamNode* paramNodes_;
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
        paramNodes_ = NULL;
    }

    ParamNode* paramNodes(){return paramNodes_;}

    virtual void storeState(Archive& archive)
    {
        if(paramNodes_ != NULL) paramNodes_->storeState(archive);
    }
    virtual void restoreState(const Archive& archive)
    {
        if(paramNodes_ != NULL) paramNodes_->restoreState(archive);
    }

    virtual std::string getParamString()
    {
        std::stringstream ss;
        if(saveParameterInFileNameCheck_.isChecked() && paramNodes_ != NULL) ss << paramNodes_->getParamString();
        return ss.str();
    }
};

}
