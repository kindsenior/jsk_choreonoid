/**
   @author Kunio Kojima
*/
#include "MultiContactStabilizerBar.h"

using namespace cnoid;

MultiContactStabilizerSetupDialog::MultiContactStabilizerSetupDialog()
{
    setWindowTitle("Multi Contact Stabilizer Setup");

    vbox = new QVBoxLayout();

    QHBoxLayout* hbox = newRow(vbox);
    hbox->addWidget(new QLabel("CM:"));
    errorCMWeightSpin.setDecimals(4);
    errorCMWeightSpin.setRange(0.0001, 10000);
    errorCMWeightSpin.setSingleStep(0.0001);
    errorCMWeightSpin.setValue(1000);
    hbox->addWidget(&errorCMWeightSpin);
            
    hbox->addSpacing(8);
    hbox->addWidget(new QLabel("Momentum:"));
    errorMomentumWeightSpin.setDecimals(4);
    errorMomentumWeightSpin.setRange(0.0001, 10000);
    errorMomentumWeightSpin.setSingleStep(0.0001);
    errorMomentumWeightSpin.setValue(100);
    hbox->addWidget(&errorMomentumWeightSpin);
            
    hbox->addSpacing(8);
    hbox->addWidget(new QLabel("AngularMomentum"));
    errorAngularMomentumWeightSpin.setDecimals(4);
    errorAngularMomentumWeightSpin.setRange(0.0001, 10000);
    errorAngularMomentumWeightSpin.setSingleStep(0.0001);
    errorAngularMomentumWeightSpin.setValue(1000);
    hbox->addWidget(&errorAngularMomentumWeightSpin);
    hbox->addStretch();


    hbox = newRow(vbox);
    hbox->addWidget(new QLabel("Force:"));
    inputForceWeightSpin.setDecimals(4);
    inputForceWeightSpin.setRange(0.0001, 10000);
    inputForceWeightSpin.setSingleStep(0.0001);
    inputForceWeightSpin.setValue(0.001);
    hbox->addWidget(&inputForceWeightSpin);

    hbox->addSpacing(8);
    hbox->addWidget(new QLabel("Moment:"));
    inputMomentWeightSpin.setDecimals(4);
    inputMomentWeightSpin.setRange(0.0001, 10000);
    inputMomentWeightSpin.setSingleStep(0.0001);
    inputMomentWeightSpin.setValue(10000);
    hbox->addWidget(&inputMomentWeightSpin);
    hbox->addStretch();


    hbox = newRow(vbox);
    hbox->addWidget(new QLabel("Window:"));
    numWindowsSpin.setDecimals(0);
    numWindowsSpin.setRange(1, 1000);
    numWindowsSpin.setSingleStep(1);
    numWindowsSpin.setValue(13);
    hbox->addWidget(&numWindowsSpin);
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
    archive.write("errorCMWeight", errorCMWeightSpin.value());
    archive.write("errorMomentumWeight", errorMomentumWeightSpin.value());
    archive.write("errorAngularMomentumWeight", errorAngularMomentumWeightSpin.value());
    archive.write("inputForceWeight", inputForceWeightSpin.value());
    archive.write("inputMomentWeight", inputMomentWeightSpin.value());
    archive.write("numWindows", numWindowsSpin.value());
    archive.write("saveParameterInFileName", saveParameterInFileNameCheck.isChecked());
}

void MultiContactStabilizerSetupDialog::restoreState(const Archive& archive)
{
    errorCMWeightSpin.setValue(archive.get("errorCMWeight", errorCMWeightSpin.value()));
    errorMomentumWeightSpin.setValue(archive.get("errorMomentumWeight", errorMomentumWeightSpin.value()));
    errorAngularMomentumWeightSpin.setValue(archive.get("errorAngularMomentumWeight", errorAngularMomentumWeightSpin.value()));
    inputForceWeightSpin.setValue(archive.get("inputForceWeight", inputForceWeightSpin.value()));
    inputMomentWeightSpin.setValue(archive.get("inputMomentWeight", inputMomentWeightSpin.value()));
    numWindowsSpin.setValue(archive.get("numWindows", numWindowsSpin.value()));
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
