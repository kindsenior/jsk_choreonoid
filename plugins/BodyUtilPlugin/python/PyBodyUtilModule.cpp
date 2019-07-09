/**
   @author Noriaki Takasugi
*/

#include <cnoid/PyUtil>
#include "../BodyUtil.h"

using namespace boost::python;
using namespace cnoid;

namespace cnoid {
    void exportPyTypesPlugin();
}

BOOST_PYTHON_MODULE(BodyUtil)
{
    exportPyTypesPlugin();
    def("links", links);
    def("jointList", jointList);
    def("angleVector", angleVector);

    class_< DrawInterface, DrawInterfacePtr, bases<Referenced> >("DrawInterface", init<Vector3f>())
        .def("setLineWidth", &DrawInterface::setLineWidth)
        .def("setColor", &DrawInterface::setColor)
        .def("show", &DrawInterface::show)
        .def("hide", &DrawInterface::hide)
        .def("drawLine", &DrawInterface::drawLine)
        .def("drawArc", &DrawInterface::drawArc)
        .def("drawArrow", &DrawInterface::drawArrow)
        .def("drawArcArrow", &DrawInterface::drawArcArrow)
        .def("drawLineArcArrow", &DrawInterface::drawLineArcArrow)
        .def("drawArrowTip", &DrawInterface::drawArrowTip)
        ;
}
