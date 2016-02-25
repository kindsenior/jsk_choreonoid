#include "PreviewControlBar.h"

using namespace cnoid;

PreviewControlSetupDialog::PreviewControlSetupDialog()
    : Dialog()
{
}

PreviewControlBar::PreviewControlBar(PreviewControlPlugin* plugin)
    : ToolBar("PreviewControl")
{
    dialog = new PreviewControlSetupDialog();

    addButton("PreviewControl")->sigClicked().connect(boost::bind(&PreviewControlPlugin::execControl, plugin));
    addButton(QIcon(":/Base/icons/setup.png"))->sigClicked().connect(boost::bind(&PreviewControlSetupDialog::show, dialog));

    setVisibleByDefault(true);
};
