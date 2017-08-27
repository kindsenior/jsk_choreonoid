set(target PyTestPlugin)
add_cnoid_python_module(${target} PyTestPluginModule.cpp)
target_link_libraries(${target} boost_python)
