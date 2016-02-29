#include "PreviewControlBar.h"

using namespace cnoid;

PreviewControlSetupDialog::PreviewControlSetupDialog()
    : ParamSetupDialog()
{
    setWindowTitle("Preview Control Setup");

    QVBoxLayout* vbox = new QVBoxLayout();

    controlModeCombo = new ComboParamWidget();
    paramWidgetVec.push_back(controlModeCombo);

    QHBoxLayout* hbox = newRow(vbox);
    controlModeCombo->setText("Control Mode:");
    controlModeCombo->setSaveName("");
    controlModeCombo->setArchiveName("controlMode");
    controlModeCombo->insertItem(DynamicsFilter, QString("DynamicsFilter"));
    controlModeCombo->insertItem(TrajectoryPlanning, QString("TrajectoryPlanning"));
    controlModeCombo->setCurrentIndex(DynamicsFilter);
    controlModeCombo->addToLayout(hbox);
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

PreviewControlBar::PreviewControlBar(PreviewControlPlugin* plugin)
    : SetupToolBar("PreviewControl")
{
    initialize(plugin);
}

void PreviewControlBar::initialize(PreviewControlPlugin* plugin)
{
    dialog = (PreviewControlSetupDialog*) new PreviewControlSetupDialog();
    SetupToolBar::initialize(dialog);

    addButton("PreviewControl")->sigClicked().connect(boost::bind(&PreviewControlPlugin::execControl, plugin));
    addButton(QIcon(":/Base/icons/setup.png"))->sigClicked().connect(boost::bind(&PreviewControlSetupDialog::show, dialog));

    setVisibleByDefault(true);
}
