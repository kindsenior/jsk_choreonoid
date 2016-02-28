#include "PreviewControlBar.h"

using namespace cnoid;

PreviewControlSetupDialog::PreviewControlSetupDialog()
    : ParamSetupDialog()
{
    setWindowTitle("Preview Control Setup");

    QVBoxLayout* vbox = new QVBoxLayout();

    modeCombo = new ComboParamWidget();
    paramWidgetVec.push_back(modeCombo);

    QHBoxLayout* hbox = newRow(vbox);
    modeCombo->setText("Control Mode:");
    modeCombo->setSaveName("");
    modeCombo->setArchiveName("controlMode");
    modeCombo->insertItem(DynamicsFilter, QString("DynamicsFilter"));
    modeCombo->insertItem(TrajectoryPlanning, QString("TrajectoryPlanning"));
    modeCombo->setCurrentIndex(DynamicsFilter);
    modeCombo->addToLayout(hbox);
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
