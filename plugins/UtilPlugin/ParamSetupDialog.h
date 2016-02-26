/**
   @author Kunio Kojima
*/
#pragma once

#include <cnoid/Archive>
#include "ParamWidget.h"

namespace cnoid {

class ParamSetupDialog : public Dialog
{
protected:
    std::vector<ParamWidget*> paramWidgetVec;

public:
    CheckBox saveParameterInFileNameCheck;

    ParamSetupDialog() : Dialog() {};

    QHBoxLayout* newRow(QVBoxLayout* vbox)
    {
        QHBoxLayout* hbox = new QHBoxLayout();
        hbox->setSpacing(2);
        hbox->setContentsMargins(2, 2, 2, 2);
        vbox->addLayout(hbox);
        return hbox;
    }

    // void addSeparator(QVBoxLayout* vbox)
    // {
    //     vbox->addSpacing(4);
    //     vbox->addWidget(new HSeparator());
    //     vbox->addSpacing(2);
    // }

    // void addSeparator(QVBoxLayout* vbox, QWidget* widget)
    // {
    //     vbox->addSpacing(4);
    //     vbox->addLayout(new HSeparatorBox(widget));
    //     vbox->addSpacing(2);
    // }

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

    virtual void storeState(Archive& archive)
    {
        for(std::vector<ParamWidget*>::iterator iter = paramWidgetVec.begin(); iter != paramWidgetVec.end(); ++iter){
            archive.write((*iter)->archiveName(), (*iter)->getParam());
        }
        archive.write("saveParameterInFileName", saveParameterInFileNameCheck.isChecked());
    }

    virtual void restoreState(const Archive& archive)
    {
        for(std::vector<ParamWidget*>::iterator iter = paramWidgetVec.begin(); iter != paramWidgetVec.end(); ++iter){
            (*iter)->setParam(archive.get((*iter)->archiveName(), (*iter)->getParam()));
        }
        saveParameterInFileNameCheck.setChecked(archive.get("saveParameterInFileName", saveParameterInFileNameCheck.isChecked()));
    }
};

}
