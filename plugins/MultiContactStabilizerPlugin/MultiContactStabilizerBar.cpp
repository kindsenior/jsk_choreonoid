/**
   @author Kunio Kojima
*/
#include "MultiContactStabilizerBar.h"

using namespace cnoid;

MultiContactStabilizerSetupDialog::MultiContactStabilizerSetupDialog()
{
    setWindowTitle("Multi Contact Stabilizer Setup");

    vbox = new QVBoxLayout();

    numWindowsSpin = new SpinParamWidget();
    paramWidgetVec.push_back(numWindowsSpin);
    errorCMWeightSpin = new SpinParamWidget();
    paramWidgetVec.push_back(errorCMWeightSpin);
    errorMomentumWeightSpin = new SpinParamWidget();
    paramWidgetVec.push_back(errorMomentumWeightSpin);
    errorAngularMomentumWeightSpin = new SpinParamWidget();
    paramWidgetVec.push_back(errorAngularMomentumWeightSpin);
    inputForceWeightSpin = new SpinParamWidget();
    paramWidgetVec.push_back(inputForceWeightSpin);
    inputMomentWeightSpin = new SpinParamWidget();
    paramWidgetVec.push_back(inputMomentWeightSpin);

    QHBoxLayout* hbox = newRow(vbox);
    errorCMWeightSpin->setText("CM:");
    errorCMWeightSpin->setSaveName("CM");
    errorCMWeightSpin->setArchiveName("errorCMWeight");
    errorCMWeightSpin->setValue(100);
    errorCMWeightSpin->addToLayout(hbox);
            
    hbox->addSpacing(8);
    errorMomentumWeightSpin->setText("Momentum:");
    errorMomentumWeightSpin->setSaveName("P");
    errorMomentumWeightSpin->setArchiveName("errorMomentumWeight");
    errorMomentumWeightSpin->setValue(10);
    errorMomentumWeightSpin->addToLayout(hbox);
            
    hbox->addSpacing(8);
    errorAngularMomentumWeightSpin->setText("AngularMomentum:");
    errorAngularMomentumWeightSpin->setSaveName("L");
    errorAngularMomentumWeightSpin->setArchiveName("errorAngularMomentumWeight");
    errorAngularMomentumWeightSpin->setValue(1000);
    errorAngularMomentumWeightSpin->addToLayout(hbox);
    hbox->addStretch();


    hbox = newRow(vbox);
    inputForceWeightSpin->setText("Force:");
    inputForceWeightSpin->setArchiveName("inputForceWeight");
    inputForceWeightSpin->setSaveName("f");
    inputForceWeightSpin->setValue(0.001);
    inputForceWeightSpin->addToLayout(hbox);

    hbox->addSpacing(8);
    inputMomentWeightSpin->setText("Moment:");
    inputMomentWeightSpin->setSaveName("n");
    inputMomentWeightSpin->setArchiveName("inputMomentWeight");
    inputMomentWeightSpin->setValue(10);
    inputMomentWeightSpin->addToLayout(hbox);
    hbox->addStretch();


    hbox = newRow(vbox);
    numWindowsSpin->setText("Window:");
    numWindowsSpin->setSaveName("N");
    numWindowsSpin->setArchiveName("numWindows");
    numWindowsSpin->setDecimals(0);
    numWindowsSpin->setRange(1, 1000);
    numWindowsSpin->setSingleStep(1);
    numWindowsSpin->setValue(13);
    numWindowsSpin->addToLayout(hbox);
    hbox->addStretch();


    hbox = newRow(vbox);
    saveParameterInFileNameCheck.setText("Save parameters in file name");
    saveParameterInFileNameCheck.setChecked(true);
    hbox->addWidget(&saveParameterInFileNameCheck);
    hbox->addStretch();

    QVBoxLayout* topVBox = new QVBoxLayout();
    topVBox->addLayout(vbox);

    QPushButton* okButton = new QPushButton("&Ok");
    okButton->setDefault(true);
    QDialogButtonBox* buttonBox = new QDialogButtonBox(this);
    buttonBox->addButton(okButton, QDialogButtonBox::AcceptRole);
    connect(buttonBox,SIGNAL(accepted()), this, SLOT(accept()));
    topVBox->addWidget(buttonBox);

    setLayout(topVBox);
}

// void MultiContactStabilizerSetupDialog::addSeparator(QVBoxLayout* vbox)
// {
//     vbox->addSpacing(4);
//     vbox->addWidget(new HSeparator());
//     vbox->addSpacing(2);
// }

// void MultiContactStabilizerSetupDialog::addSeparator(QVBoxLayout* vbox, QWidget* widget)
// {
//     vbox->addSpacing(4);
//     vbox->addLayout(new HSeparatorBox(widget));
//     vbox->addSpacing(2);
// }

QHBoxLayout* MultiContactStabilizerSetupDialog::newRow(QVBoxLayout* vbox)
{
    QHBoxLayout* hbox = new QHBoxLayout();
    hbox->setSpacing(2);
    hbox->setContentsMargins(2, 2, 2, 2);
    vbox->addLayout(hbox);
    return hbox;
}

void MultiContactStabilizerSetupDialog::storeState(Archive& archive)
{
    for(std::vector<ParamWidget*>::iterator iter = paramWidgetVec.begin(); iter != paramWidgetVec.end(); ++iter){
        archive.write((*iter)->archiveName(), (*iter)->getParam());
    }
    archive.write("saveParameterInFileName", saveParameterInFileNameCheck.isChecked());
}

void MultiContactStabilizerSetupDialog::restoreState(const Archive& archive)
{
    for(std::vector<ParamWidget*>::iterator iter = paramWidgetVec.begin(); iter != paramWidgetVec.end(); ++iter){
        (*iter)->setParam(archive.get((*iter)->archiveName(), (*iter)->getParam()));
    }
    saveParameterInFileNameCheck.setChecked(archive.get("saveParameterInFileName", saveParameterInFileNameCheck.isChecked()));
}

MultiContactStabilizerBar::MultiContactStabilizerBar(MultiContactStabilizerPlugin* plugin)
    : ToolBar("MultiContactStabilizerBar")
{
    dialog = new MultiContactStabilizerSetupDialog();

    addButton("MCS")->sigClicked().connect(boost::bind(&MultiContactStabilizerPlugin::execControl, plugin));
    addButton(QIcon(":/Base/icons/setup.png"))->sigClicked().connect(boost::bind(&MultiContactStabilizerSetupDialog::show, dialog));

    setVisibleByDefault(true);// 効かない?
}

bool MultiContactStabilizerBar::storeState(Archive& archive)
{
    dialog->storeState(archive);
    return true;
}

bool MultiContactStabilizerBar::restoreState(const Archive& archive)
{
    dialog->restoreState(archive);
    return true;
}
