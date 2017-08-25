/**
   @author Shin'ichiro Nakaoka
*/

#include <cnoid/Plugin>
#include <cnoid/MenuManager>
#include <cnoid/MessageView>

using namespace std;
using namespace cnoid;

class TestPlugin : public Plugin
{
public:

    TestPlugin() : Plugin("Test")
    {

    }

    virtual bool initialize()
    {
        Action* menuItem = menuManager().setPath("/View").addItem("Test");
        menuItem->sigTriggered().connect(bind(&TestPlugin::onTestTriggered, this));
        return true;
    }

private:

    void onTestTriggered()
    {
        MessageView::instance()->putln("Test !");
    }
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(TestPlugin)
