/**
   @author Kunio Kojima
*/
#include "CascadedControlBar.h"

using namespace cnoid;

CascadedControlSetupLayout::CascadedControlSetupLayout()
    : ParamSetupLayout()
{
    paramNodes_ = new ParamVector();
    paramNodes_->setArchiveName("CC");

    // // in order for saving param to file name
    // MultiContactStabilizerSetupLayout* parentMcsLayout = new MultiContactStabilizerSetupLayout();
    // paramNodes_->addParamNode(parentMcsLayout->paramNodes());
    // MultiContactStabilizerSetupLayout* childMcsLayout = new MultiContactStabilizerSetupLayout();
    // paramNodes_->addParamNode(childMcsLayout->paramNodes());

    // // in order of param setup dialog
    // this->addLayout(parentMcsLayout);
    // this->addLayout(childMcsLayout);


    // in order for saving param to file name
    SlideFrictionControlSetupLayout* parentSfcLayout = new SlideFrictionControlSetupLayout();
    paramNodes_->addParamNode(parentSfcLayout->paramNodes());
    SlideFrictionControlSetupLayout* childSfcLayout = new SlideFrictionControlSetupLayout();
    paramNodes_->addParamNode(childSfcLayout->paramNodes());

    // in order of param setup dialog
    this->addLayout(parentSfcLayout);
    this->addLayout(childSfcLayout);

    QHBoxLayout* hbox = newRow();
    saveParameterInFileNameCheck_.setText("Save parameters in file name");
    saveParameterInFileNameCheck_.setChecked(true);
    hbox->addWidget(&saveParameterInFileNameCheck_);
    hbox->addStretch();
}

CascadedControlSetupDialog::CascadedControlSetupDialog()
    : ParamSetupDialog()
{
    setWindowTitle("Multi Contact Stabilizer Setup");

    QVBoxLayout* vbox = new QVBoxLayout();

    layout_ = (ParamSetupLayout*) new CascadedControlSetupLayout();
    vbox->addLayout(layout_);

    QPushButton* okButton = new QPushButton("&Ok");
    okButton->setDefault(true);
    QDialogButtonBox* buttonBox = new QDialogButtonBox(this);
    buttonBox->addButton(okButton, QDialogButtonBox::AcceptRole);
    connect(buttonBox,SIGNAL(accepted()), this, SLOT(accept()));
    vbox->addWidget(buttonBox);

    setLayout(vbox);
}

CascadedControlBar::CascadedControlBar(CascadedControlPlugin* plugin)
    : SetupToolBar("CascadedControlBar")
{
    initialize(plugin);
}

void CascadedControlBar::initialize(CascadedControlPlugin* plugin)
{
    dialog = (CascadedControlSetupDialog*) new CascadedControlSetupDialog();
    SetupToolBar::initialize(dialog);

    addButton("CC")->sigClicked().connect(boost::bind(&CascadedControlPlugin::execControl, plugin));
    addButton(QIcon(":/Base/icons/setup.png"))->sigClicked().connect(boost::bind(&CascadedControlSetupDialog::show, dialog));

    setVisibleByDefault(true);// 効かない?
}
