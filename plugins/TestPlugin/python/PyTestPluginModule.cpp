
#include <iostream>
#include <cnoid/PyUtil>

using namespace boost::python;

void test()
{
  std::cout << "This is Test Plugin" << std::endl;
}

BOOST_PYTHON_MODULE(TestPlugin)
{
  def("test", test);
}
