/**
   @author Kunio Kojima
*/
#pragma once

#include <sstream>
#include <vector>
#include <iostream>

#include <boost/algorithm/string.hpp>
#include <boost/bind.hpp>

#include <cnoid/ToolBar>
#include <cnoid/Dialog>
#include <cnoid/SpinBox>
// #include <cnoid/Separator>
#include <cnoid/Buttons>
#include <cnoid/CheckBox>
#include <cnoid/LineEdit>
#include <cnoid/ComboBox>
#include <QDialogButtonBox>

#include "ParamNode.h"

namespace cnoid {

class ParamWidget : public CheckBox,
                    public ParamNode
{
public:
    ParamWidget()
        : CheckBox(),
          ParamNode()
    {
        setText("default text");
        setChecked(true);
    }

    void storeState(Archive& archive)
    {
        // std::string archiveName = archiveName();// no mathing function call?
        archive.write(archiveName_, getParam());
        archive.write(archiveName_+"Check", isChecked());
    };
    void restoreState(const Archive& archive)
    {
        // std::string archiveName = archiveName();// no matching function call?
        setParam(archive.get(archiveName_, getParam()));
        setChecked(archive.get(archiveName_+"Check", isChecked()));
    };

    void addToLayout(QBoxLayout* layout)
    {
        layout->addWidget((CheckBox*)this);
    }

    std::string getParamString()
    {
        std::stringstream ss;
        if(isChecked()){
            ss << "_" << getParam() << saveName();
        }
        return ss.str();
    }

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
        setRange(0.0001, 1000000);
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

class SpinVectorParamWidget: public ParamWidget,
                             public std::vector<SpinBox*>
{
protected:
    int maxRange;
    QBoxLayout* parentLayout;
    PushButton* pulseButton;
    PushButton* minusButton;

    void addSpinBoxWidget(int x)
    {
        SpinBox* sb = new SpinBox();
        sb->setRange(0,maxRange);
        sb->setValue(x);
        this->push_back(sb);
        if(parentLayout != NULL) parentLayout->addWidget(sb);
    }

    void removeSpinBoxWidget()
    {
        delete this->back();
        this->pop_back();
    }

    void removeAllSpinWidet()
    {
        for(std::vector<SpinBox*>::iterator iter = this->begin(); iter != this->end(); ++iter){
            // parentLayout->removeWidget((SpinBox*) *iter);// removeWidget()できない Qtのバグ?
            delete *iter;
        }
    }

public:
    SpinVectorParamWidget()
        :ParamWidget(),
         std::vector<SpinBox*>()
    {
        maxRange = 1000;
        parentLayout = NULL;

        pulseButton = new PushButton("+");
        pulseButton->sigClicked().connect(boost::bind(&SpinVectorParamWidget::addSpinBoxWidget, this, 1));
        pulseButton->setFixedWidth(20);

        minusButton = new PushButton("-");
        minusButton->sigClicked().connect(boost::bind(&SpinVectorParamWidget::removeSpinBoxWidget, this));
        minusButton->setFixedWidth(20);
    }

    void addToLayout(QBoxLayout* layout)
    {
        parentLayout = layout;
        ParamWidget::addToLayout(layout);
        parentLayout->addWidget((PushButton*) pulseButton);
        parentLayout->addWidget((QPushButton*) minusButton);
        for(std::vector<SpinBox*>::iterator iter = this->begin(); iter != this->end(); ++iter) parentLayout->addWidget((SpinBox*) *iter);
    }

    std::string getParam()
    {
        std::stringstream ss;
        std::vector<SpinBox*>::iterator iter = this->begin();
        while(true){
            ss << (*iter)->value();
            if(++iter == this->end()) break;
            else ss << ",";
        }
        return ss.str();
    }
    void setParam(std::string param)
    {
        removeAllSpinWidet();
        std::vector<SpinBox*>::clear();
        std::vector<std::string> stringVec;
        boost::algorithm::split(stringVec, param, boost::is_any_of(","));
        for(std::vector<std::string>::iterator iter = stringVec.begin(); iter != stringVec.end(); ++iter){
            addSpinBoxWidget(std::atoi(iter->c_str()));
        }
    }

    void setValue(std::vector<int> vec)
    {
        removeAllSpinWidet();
        std::vector<SpinBox*>::clear();
        for(std::vector<int>::iterator iter = vec.begin(); iter != vec.end(); ++iter){
            addSpinBoxWidget(*iter);
        }
    }

    std::vector<int> value()
    {
        std::vector<int> ret;
        for(std::vector<SpinBox*>::iterator iter = this->begin(); iter != this->end(); ++iter){
            ret.push_back((*iter)->value());
        }
        return ret;
    }
};

}
