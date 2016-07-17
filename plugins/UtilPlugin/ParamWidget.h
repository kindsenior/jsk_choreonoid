/**
   @author Kunio Kojima
*/
#pragma once

#include <sstream>
#include <vector>
#include <iostream>

#include <boost/algorithm/string.hpp>

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
        setChecked(true);
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
    // std::vector<SpinBox*> spinBoxVector;//ポインタじゃないとerror: use of deleted function
    int maxRange;
    QBoxLayout* parentLayout;

    void addAllSpinWidget()
    {
        if(parentLayout){
            // for(std::vector<SpinBox*>::iterator iter = spinBoxVector.begin(); iter != spinBoxVector.end(); ++iter){
            //     layout->addWidget((SpinBox*) *iter);
            // }
            for(std::vector<SpinBox*>::iterator iter = this->begin(); iter != this->end(); ++iter){
                parentLayout->addWidget((SpinBox*) *iter);
            }
        }
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
    }

    void addToLayout(QBoxLayout* layout)
    {
        parentLayout = layout;
        ParamWidget::addToLayout(layout);
        addAllSpinWidget();
    }

    std::string getParam()
    {
        std::stringstream ss;
        // for(std::vector<SpinBox*>::iterator iter = spinBoxVector.begin(); iter != spinBoxVector.end(); ++iter){
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
        // spinBoxVector.clear();
        removeAllSpinWidet();
        // this->clear();
        std::vector<SpinBox*>::clear();
        std::vector<std::string> stringVec;
        boost::algorithm::split(stringVec, param, boost::is_any_of(","));
        for(std::vector<std::string>::iterator iter = stringVec.begin(); iter != stringVec.end(); ++iter){
            SpinBox* sb = new SpinBox();
            sb->setRange(0,maxRange);
            sb->setValue(std::atoi(iter->c_str()));
            // spinBoxVector.push_back(sb);
            this->push_back(sb);
        }
        // updateAllWidget();
        addAllSpinWidget();
    }

    void setValue(std::vector<int> vec)
    {
        // spinBoxVector.clear();
        // this->clear();
        std::vector<SpinBox*>::clear();
        for(std::vector<int>::iterator iter = vec.begin(); iter != vec.end(); ++iter){
            SpinBox* sb = new SpinBox();
            sb->setRange(0,maxRange);
            sb->setValue(*iter);
            // spinBoxVector.push_back(sb);
            this->push_back(sb);
        }
        addAllSpinWidget();
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
