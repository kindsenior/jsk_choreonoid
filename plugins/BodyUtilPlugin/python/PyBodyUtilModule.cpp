

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
}
