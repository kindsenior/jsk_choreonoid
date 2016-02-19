/**
   @author Kunio Kojima
*/
#pragma once

#include "ParamWidget.h"

namespace cnoid {

class ParamSetupDialog : public Dialog
{
protected:
    std::vector<ParamWidget*> paramWidgetVec;

public:
    ParamSetupDialog() : Dialog() {};

    QHBoxLayout* newRow(QVBoxLayout* vbox)
    {
        QHBoxLayout* hbox = new QHBoxLayout();
        hbox->setSpacing(2);
        hbox->setContentsMargins(2, 2, 2, 2);
        vbox->addLayout(hbox);
        return hbox;
    }

    std::string getParamString()
    {
        std::stringstream ss;
        for(std::vector<ParamWidget*>::iterator iter = paramWidgetVec.begin(); iter != paramWidgetVec.end(); ++iter){
            ss << "_" << (*iter)->getParam() << (*iter)->saveName();
        }
        std::string str = ss.str();
        replace(str.begin(), str.end(), '.', '-');
        return str;
    }

    void storeState(Archive& archive){
        for(std::vector<ParamWidget*>::iterator iter = paramWidgetVec.begin(); iter != paramWidgetVec.end(); ++iter){
            archive.write((*iter)->archiveName(), (*iter)->getParam());
        }
    }

    void restoreState(const Archive& archive)
    {
        for(std::vector<ParamWidget*>::iterator iter = paramWidgetVec.begin(); iter != paramWidgetVec.end(); ++iter){
            (*iter)->setParam(archive.get((*iter)->archiveName(), (*iter)->getParam()));
        }
    }
};

}
