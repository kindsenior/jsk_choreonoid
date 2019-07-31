/**
   @author Noriaki Takasugi
*/

#include <iostream>
#include <cnoid/PyUtil>

#include <pybind11/pybind11.h>

using namespace cnoid;
namespace py = pybind11;

void test()
{
  std::cout << "This is Test Plugin" << std::endl;
}

PYBIND11_MODULE(TestPlugin, m)
{
  m.def("test", test);
}
