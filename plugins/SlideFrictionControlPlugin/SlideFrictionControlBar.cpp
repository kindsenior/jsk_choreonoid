/**
   @author Kunio Kojima
*/
#include "SlideFrictionControlBar.h"

using namespace cnoid;

SlideFrictionControlSetupLayout::SlideFrictionControlSetupLayout()
    : ParamSetupLayout()
{
    paramNodes_ = new ParamMap();
    paramNodes_->setArchiveName("SFC");

    // in order for saving param to file name
    takeoffPhaseRatioSpinArray = new SpinArrayParamWidget<double>(5);
    paramNodes_->addParamNode(takeoffPhaseRatioSpinArray);
    landingPhaseRatioSpinArray = new SpinArrayParamWidget<double>(5);
    paramNodes_->addParamNode(landingPhaseRatioSpinArray);
    errorCMWeightSpin = new SpinParamWidget();
    paramNodes_->addParamNode(errorCMWeightSpin);
    errorMomentumWeightSpin = new SpinParamWidget();
    paramNodes_->addParamNode(errorMomentumWeightSpin);
    errorAngularMomentumWeightSpin = new SpinParamWidget();
    paramNodes_->addParamNode(errorAngularMomentumWeightSpin);
    errorYawAngularMomentumWeightSpin = new SpinParamWidget();
    paramNodes_->addParamNode(errorYawAngularMomentumWeightSpin);
    inputForceWeightSpin = new SpinParamWidget();
    paramNodes_->addParamNode(inputForceWeightSpin);
    inputMomentWeightSpin = new SpinParamWidget();
    paramNodes_->addParamNode(inputMomentWeightSpin);
    inputYawMomentWeightSpin = new SpinParamWidget();
    paramNodes_->addParamNode(inputYawMomentWeightSpin);
    blockSpinVec = new SpinVectorParamWidget<int>();
    paramNodes_->addParamNode(blockSpinVec);
    xDivisionNumSpin = new SpinParamWidget();
    paramNodes_->addParamNode(xDivisionNumSpin);
    yDivisionNumSpin = new SpinParamWidget();
    paramNodes_->addParamNode(yDivisionNumSpin);
    dtSpin = new SpinParamWidget();
    paramNodes_->addParamNode(dtSpin);

    // in order of param setup dialog
    QHBoxLayout* hbox = newRow();
    takeoffPhaseRatioSpinArray->setText("takeff phase ratio:");
    takeoffPhaseRatioSpinArray->setSaveName("TO");
    takeoffPhaseRatioSpinArray->setArchiveName("takeoffPhaseRatioList");
    std::vector<double> ratioVec{1,2,1,2,1};
    takeoffPhaseRatioSpinArray->setValue(ratioVec);
    takeoffPhaseRatioSpinArray->addToLayout(hbox);

    hbox->addSpacing(15);
    landingPhaseRatioSpinArray->setText("landing phase ratio:");
    landingPhaseRatioSpinArray->setSaveName("LA");
    landingPhaseRatioSpinArray->setArchiveName("landingPhaseRatioList");
    landingPhaseRatioSpinArray->setValue(ratioVec);
    landingPhaseRatioSpinArray->addToLayout(hbox);
    hbox->addStretch();

    hbox = newRow();
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

    hbox->addSpacing(15);
    errorMomentumWeightSpin->setText("Momentum:");
    errorMomentumWeightSpin->setSaveName("P");
    errorMomentumWeightSpin->setArchiveName("errorMomentumWeight");
    errorMomentumWeightSpin->setValue(10);
    errorMomentumWeightSpin->addToLayout(hbox);

    hbox->addSpacing(15);
    errorAngularMomentumWeightSpin->setText("AngularMomentum:");
    errorAngularMomentumWeightSpin->setSaveName("L");
    errorAngularMomentumWeightSpin->setArchiveName("errorAngularMomentumWeight");
    errorAngularMomentumWeightSpin->setValue(1000);
    errorAngularMomentumWeightSpin->addToLayout(hbox);

    hbox->addSpacing(15);
    errorYawAngularMomentumWeightSpin->setText("YawAngularMomentum:");
    errorYawAngularMomentumWeightSpin->setSaveName("Lz");
    errorYawAngularMomentumWeightSpin->setArchiveName("errorYawAngularMomentumWeight");
    errorYawAngularMomentumWeightSpin->setValue(1);
    errorYawAngularMomentumWeightSpin->addToLayout(hbox);
    hbox->addStretch();

    hbox = newRow();
    inputForceWeightSpin->setText("Force:");
    inputForceWeightSpin->setArchiveName("inputForceWeight");
    inputForceWeightSpin->setSaveName("f");
    inputForceWeightSpin->setValue(0.001);
    inputForceWeightSpin->addToLayout(hbox);

    hbox->addSpacing(15);
    inputMomentWeightSpin->setText("Moment:");
    inputMomentWeightSpin->setSaveName("n");
    inputMomentWeightSpin->setArchiveName("inputMomentWeight");
    inputMomentWeightSpin->setValue(10);
    inputMomentWeightSpin->addToLayout(hbox);

    hbox->addSpacing(15);
    inputYawMomentWeightSpin->setText("Yaw Moment:");
    inputYawMomentWeightSpin->setSaveName("nz");
    inputYawMomentWeightSpin->setArchiveName("inputYawMomentWeight");
    inputYawMomentWeightSpin->setValue(1);
    inputYawMomentWeightSpin->addToLayout(hbox);
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
    xDivisionNumSpin->setText("x Division Number:");
    xDivisionNumSpin->setSaveName("x");
    xDivisionNumSpin->setArchiveName("xDivisionNum");
    xDivisionNumSpin->setValue(2);
    xDivisionNumSpin->setDecimals(0);
    xDivisionNumSpin->setRange(1,100);
    xDivisionNumSpin->setSingleStep(1);
    xDivisionNumSpin->addToLayout(hbox);

    hbox->addSpacing(15);
    yDivisionNumSpin->setText("y Division Number:");
    yDivisionNumSpin->setSaveName("y");
    yDivisionNumSpin->setArchiveName("yDivisionNum");
    yDivisionNumSpin->setValue(2);
    yDivisionNumSpin->setDecimals(0);
    yDivisionNumSpin->setRange(1,100);
    yDivisionNumSpin->setSingleStep(1);
    yDivisionNumSpin->addToLayout(hbox);
    hbox->addStretch();

    hbox = newRow();
    saveParameterInFileNameCheck_.setText("Save parameters in file name");
    saveParameterInFileNameCheck_.setChecked(true);
    hbox->addWidget(&saveParameterInFileNameCheck_);
    hbox->addStretch();
}

SlideFrictionControlSetupDialog::SlideFrictionControlSetupDialog()
    : ParamSetupDialog()
{
    setWindowTitle("Slide Friction Control Setup");

    QVBoxLayout* vbox = new QVBoxLayout();

    layout_ = (ParamSetupLayout*) new SlideFrictionControlSetupLayout();
    vbox->addLayout(layout_);

    QPushButton* okButton = new QPushButton("&Ok");
    okButton->setDefault(true);
    QDialogButtonBox* buttonBox = new QDialogButtonBox(this);
    buttonBox->addButton(okButton, QDialogButtonBox::AcceptRole);
    connect(buttonBox,SIGNAL(accepted()), this, SLOT(accept()));
    vbox->addWidget(buttonBox);

    setLayout(vbox);
}

SlideFrictionControlBar::SlideFrictionControlBar(SlideFrictionControlPlugin* plugin)
    : SetupToolBar("SlideFrictionControlBar")
{
    initialize(plugin);
}

void SlideFrictionControlBar::initialize(SlideFrictionControlPlugin* plugin)
{
    dialog = (SlideFrictionControlSetupDialog*) new SlideFrictionControlSetupDialog();
    SetupToolBar::initialize(dialog);

    addButton("SFC")->sigClicked().connect(boost::bind(&SlideFrictionControlPlugin::execControl, plugin));
    addButton(QIcon(":/Base/icons/setup.png"))->sigClicked().connect(boost::bind(&SlideFrictionControlSetupDialog::show, dialog));

    setVisibleByDefault(true);// 効かない?
}
