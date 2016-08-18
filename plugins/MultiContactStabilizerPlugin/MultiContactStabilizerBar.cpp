/**
   @author Kunio Kojima
*/
#include "MultiContactStabilizerBar.h"

using namespace cnoid;

MultiContactStabilizerSetupLayout::MultiContactStabilizerSetupLayout()
    : ParamSetupLayout()
{
    paramNodes_ = new ParamMap();
    paramNodes_->setArchiveName("MCS");

    // in order for saving param to file name
    errorCMWeightSpin = new SpinParamWidget();
    paramNodes_->addParamNode(errorCMWeightSpin);
    errorMomentumWeightSpin = new SpinParamWidget();
    paramNodes_->addParamNode(errorMomentumWeightSpin);
    errorAngularMomentumWeightSpin = new SpinParamWidget();
    paramNodes_->addParamNode(errorAngularMomentumWeightSpin);
    inputForceWeightSpin = new SpinParamWidget();
    paramNodes_->addParamNode(inputForceWeightSpin);
    inputMomentWeightSpin = new SpinParamWidget();
    paramNodes_->addParamNode(inputMomentWeightSpin);
    blockSpinVec = new SpinVectorParamWidget();
    paramNodes_->addParamNode(blockSpinVec);
    dtSpin = new SpinParamWidget();
    paramNodes_->addParamNode(dtSpin);

    // in order of param setup dialog
    QHBoxLayout* hbox = newRow();
    dtSpin->setRange(0.001,1);
    dtSpin->setText("dt [sec]:");
    dtSpin->setSaveName("dt");
    dtSpin->setArchiveName("dt");
    dtSpin->setValue(100);
    dtSpin->setDecimals(3);
    dtSpin->addToLayout(hbox);
    hbox->addStretch();

    hbox = newRow();
    errorCMWeightSpin->setRange(0,1000000000);
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

    hbox = newRow();
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

    hbox = newRow();
    blockSpinVec->setText("Blocking:");
    blockSpinVec->setSaveName("B");
    blockSpinVec->setArchiveName("blockingList");
    std::vector<int> vec{1,1,1,1,1,1,1,1,1,1,1,1,1};
    blockSpinVec->setValue(vec);
    blockSpinVec->addToLayout(hbox);
    hbox->addStretch();

    hbox = newRow();
    saveParameterInFileNameCheck_.setText("Save parameters in file name");
    saveParameterInFileNameCheck_.setChecked(true);
    hbox->addWidget(&saveParameterInFileNameCheck_);
    hbox->addStretch();
}

MultiContactStabilizerSetupDialog::MultiContactStabilizerSetupDialog()
    : ParamSetupDialog()
{
    setWindowTitle("Multi Contact Stabilizer Setup");

    QVBoxLayout* vbox = new QVBoxLayout();

    layout_ = (ParamSetupLayout*) new MultiContactStabilizerSetupLayout();
    vbox->addLayout(layout_);

    QPushButton* okButton = new QPushButton("&Ok");
    okButton->setDefault(true);
    QDialogButtonBox* buttonBox = new QDialogButtonBox(this);
    buttonBox->addButton(okButton, QDialogButtonBox::AcceptRole);
    connect(buttonBox,SIGNAL(accepted()), this, SLOT(accept()));
    vbox->addWidget(buttonBox);

    setLayout(vbox);
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
