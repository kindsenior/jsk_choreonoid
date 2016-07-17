/**
   @author Kunio Kojima
*/
#include "MultiContactStabilizerBar.h"

using namespace cnoid;

MultiContactStabilizerSetupDialog::MultiContactStabilizerSetupDialog()
    : ParamSetupDialog()
{
    setWindowTitle("Multi Contact Stabilizer Setup");

    QVBoxLayout* vbox = new QVBoxLayout();

    // in order for saving param to file name
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
    blockSpinVec = new SpinVectorParamWidget();
    paramWidgetVec.push_back(blockSpinVec);

    // in order of param setup dialog
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
    blockSpinVec->setText("Blocking:");
    blockSpinVec->setSaveName("B");
    blockSpinVec->setArchiveName("blockingList");
    std::vector<int> vec{1,1,1,1,1,1,1,1,1,1,1,1,1};
    blockSpinVec->setValue(vec);
    blockSpinVec->addToLayout(hbox);
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

MultiContactStabilizerBar::MultiContactStabilizerBar(MultiContactStabilizerPlugin* plugin)
    : SetupToolBar("MultiContactStabilizerBar")
{
    initialize(plugin);
}

void MultiContactStabilizerBar::initialize(MultiContactStabilizerPlugin* plugin)
{
    dialog = (MultiContactStabilizerSetupDialog*) new MultiContactStabilizerSetupDialog();
    SetupToolBar::initialize(dialog);

    addButton("MCS")->sigClicked().connect(boost::bind(&MultiContactStabilizerPlugin::execControl, plugin));
    addButton(QIcon(":/Base/icons/setup.png"))->sigClicked().connect(boost::bind(&MultiContactStabilizerSetupDialog::show, dialog));

    setVisibleByDefault(true);// 効かない?
}
