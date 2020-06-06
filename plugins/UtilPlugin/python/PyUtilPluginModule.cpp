/**
   @author Kunio Kojima
*/

#include <cnoid/PyUtil>
#include "../UtilPlugin.h"

#include <pybind11/pybind11.h>

using namespace cnoid;
namespace py = pybind11;

PYBIND11_MODULE(UtilPlugin, m)
{
    m.doc() = "Choreonoid UtilPlugin module";

    py::module::import("cnoid.Base");

    py::class_< DrawInterface, DrawInterfacePtr, Referenced >(m, "DrawInterface")
        .def(py::init<Eigen::Vector3f>())
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
