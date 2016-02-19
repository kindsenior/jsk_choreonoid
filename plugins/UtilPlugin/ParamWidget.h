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
        setDecimals(4);
        setRange(0.0001, 10000);
        setSingleStep(0.0001);
        setValue(1000);
    };

    void addToLayout(QBoxLayout* layout)
    {
        ParamWidget::addToLayout(layout);
        layout->addWidget((DoubleSpinBox*)this);
    }

    double getParam(){return DoubleSpinBox::value();}
    void setParam(double param){DoubleSpinBox::setValue(param);}
};

}
