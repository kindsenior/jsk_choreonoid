if(ONLYTEST)
	project(multi_contact_tabilizer)

	find_package(PkgConfig)

	# for eigen
  pkg_check_modules(eigen REQUIRED eigen3)
	include_directories(${eigen_INCLUDE_DIRS})

	# for qpOASES
	if(NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/qpOASES-3.0)
		execute_process(COMMAND svn co https://projects.coin-or.org/svn/qpOASES/stable/3.0 ${CMAKE_CURRENT_BINARY_DIR}/qpOASES-3.0)
		execute_process(COMMAND sed -i -e "s/qpOASES\ STATIC/qpOASES\ SHARED/g" ${CMAKE_CURRENT_BINARY_DIR}/qpOASES-3.0/CMakeLists.txt)
		execute_process(COMMAND cmake -E chdir ${CMAKE_CURRENT_BINARY_DIR}/qpOASES-3.0 make)
	endif()

	add_subdirectory(${CMAKE_CURRENT_BINARY_DIR}/qpOASES-3.0 ${CMAKE_CURRENT_BINARY_DIR}/../../3rdparty/qpOASES/qpOASES-3.0)
	link_directories(${CMAKE_CURRENT_BINARY_DIR}/qpOASES-3.0/bin)
	include_directories(${CMAKE_CURRENT_BINARY_DIR}/qpOASES-3.0/include)

	# for openhrp3
	pkg_check_modules(openhrp3 REQUIRED openhrp3.1)
	include_directories(${openhrp3_INCLUDE_DIRS})
	link_directories(${openhrp3_LIBRARY_DIRS})
	execute_process(
		COMMAND pkg-config --variable=link_shared_files openhrp3.1
		OUTPUT_VARIABLE openhrp3_link_shared_files
		OUTPUT_STRIP_TRAILING_WHITESPACE
		)
	string(REGEX REPLACE "-l" "" openhrp3_link_shared_files ${openhrp3_link_shared_files})# remove -l
	string(REGEX REPLACE " " ";" openhrp3_link_shared_files ${openhrp3_link_shared_files})# change string to list

	set(libs qpOASES ${openhrp3_link_shared_files})
endif(ONLYTEST)

add_executable(mcs-test test MultiContactStabilizer ModelPredictiveController ContactConstraint QP)
target_link_libraries(mcs-test ${libs})
