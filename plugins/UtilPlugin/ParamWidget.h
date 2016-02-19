/**
   @author Kunio Kojima
*/
#pragma once

#include <iostream>

#include <cnoid/ToolBar>
#include <cnoid/Dialog>
#include <cnoid/SpinBox>
// #include <cnoid/Separator>
#include <cnoid/Buttons>
#include <cnoid/CheckBox>
#include <cnoid/LineEdit>
#include <QDialogButtonBox>

namespace cnoid {

class ParamWidget : public CheckBox
{
private:
    std::string saveName_;

public:
    ParamWidget()
        : CheckBox()
    {
        setText("hogefuga");
    };

    void addToLayout(QBoxLayout* layout)
    {
        layout->addWidget((CheckBox*)this);
    };

    void setSaveName(const std::string& name)
    {
        saveName_ = name;
    }

    std::string saveName(){return saveName_;};
};

class SpinParamWidget : public ParamWidget,
                        public DoubleSpinBox
{
public:
    SpinParamWidget()
        : ParamWidget(),
          DoubleSpinBox()
    {
        setValue(1000);
    };

    void addToLayout(QBoxLayout* layout)
    {
        ParamWidget::addToLayout(layout);
        layout->addWidget((DoubleSpinBox*)this);
    };
};

}
