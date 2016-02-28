/**
   @author Kunio Kojima
*/
#pragma once

#include <sstream>

#include <cnoid/ToolBar>
#include <cnoid/Dialog>
#include <cnoid/SpinBox>
// #include <cnoid/Separator>
#include <cnoid/Buttons>
#include <cnoid/CheckBox>
#include <cnoid/LineEdit>
#include <cnoid/ComboBox>
#include <QDialogButtonBox>

namespace cnoid {

class ParamWidget : public CheckBox
{
private:
    std::string saveName_;
    std::string archiveName_;

public:
    ParamWidget()
        : CheckBox()
    {
        setText("default text");
        saveName_ = "defaultSaveName";
        archiveName_ = "defaultArchiveName";
    }

    void addToLayout(QBoxLayout* layout)
    {
        layout->addWidget((CheckBox*)this);
    }

    void setSaveName(const std::string& name){saveName_ = name;}
    std::string saveName(){return saveName_;}
    void setArchiveName(const std::string& name){archiveName_ = name;}
    std::string archiveName(){return archiveName_;}

    virtual std::string getParam()=0;
    virtual void setParam(std::string param)=0;
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
    }

    void addToLayout(QBoxLayout* layout)
    {
        ParamWidget::addToLayout(layout);
        layout->addWidget((DoubleSpinBox*)this);
    }

    std::string getParam()
    {
        std::stringstream ss;
        ss << DoubleSpinBox::value();
        return ss.str();
    }
    void setParam(std::string param)
    {
        std::stringstream ss;
        ss.str(param);
        double x;
        ss >> x;
        DoubleSpinBox::setValue(x);
    }
};

class ComboParamWidget : public ParamWidget,
                         public ComboBox
{
public:
    ComboParamWidget()
        : ParamWidget(),
          ComboBox()
    {
    }

    void addToLayout(QBoxLayout* layout)
    {
        ParamWidget::addToLayout(layout);
        layout->addWidget((ComboBox*)this);
    }

    std::string getParam(){return currentText().toStdString();}
    void setParam(std::string param)
    {
        int idx;
        if((idx = findText(QString(param.c_str()))) != -1) setCurrentIndex(idx);
    }
};

}
