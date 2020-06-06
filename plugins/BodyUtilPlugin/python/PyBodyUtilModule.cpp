/**
   @author Noriaki Takasugi
*/

#include <cnoid/PyUtil>
#include "../BodyUtil.h"

#include <pybind11/pybind11.h>

using namespace cnoid;
namespace py = pybind11;

PYBIND11_MODULE(BodyUtil, m)
{
    m.doc() = "Choreonoid BodyUtil module";

    py::module::import("cnoid.Util");

    m.def("links", links);
    m.def("jointList", jointList);
    m.def("angleVector", angleVector);
}
