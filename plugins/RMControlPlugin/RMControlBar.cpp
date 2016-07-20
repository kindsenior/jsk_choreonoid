/**
   @author Kunio Kojima
*/
#include "RMControlBar.h"

using namespace cnoid;

RMControlSetupDialog::RMControlSetupDialog()
    : ParamSetupDialog()
{
    setWindowTitle("Resolved Momentum Control Setup");

    QVBoxLayout* vbox = new QVBoxLayout();

    // in order for saving param to file name
    initialTrajectoryCombo = new ComboParamWidget();
    paramWidgetVec.push_back(initialTrajectoryCombo);

    // in order of param setup dialog
    QHBoxLayout* hbox = newRow(vbox);
    initialTrajectoryCombo->setText("Generate Initial Trajectory:");
    initialTrajectoryCombo->setSaveName("");
    initialTrajectoryCombo->setArchiveName("initialTrajectory");
    initialTrajectoryCombo->insertItem(Generate, QString("Generate"));
    initialTrajectoryCombo->insertItem(Load, QString("Load"));
    initialTrajectoryCombo->setCurrentIndex(0);
    initialTrajectoryCombo->addToLayout(hbox);
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

RMControlBar::RMControlBar(RMControlPlugin* plugin)
    : SetupToolBar("RMControlBar")
{
    initialize(plugin);
}

void RMControlBar::initialize(RMControlPlugin* plugin)
{
    dialog = (RMControlSetupDialog*) new RMControlSetupDialog();
    SetupToolBar::initialize(dialog);

    addButton("RMC")->sigClicked().connect(boost::bind(&RMControlPlugin::execControl, plugin));
    addButton(QIcon(":/Base/icons/setup.png"))->sigClicked().connect(boost::bind(&RMControlSetupDialog::show, dialog));

    setVisibleByDefault(true);// 効かない?
}
