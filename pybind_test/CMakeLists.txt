##############################################################################
# Project
##############################################################################

cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(pybind_test LANGUAGES C CXX VERSION 3.0.0)

##############################################################################
# Find 3rd Party Packages
##############################################################################

message(STATUS "\n\n====== Finding 3rd Party Packages ======\n")

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_pytest REQUIRED)

find_package(maliput REQUIRED)
find_package(drake_vendor REQUIRED)

##############################################################################
# Project Configuration
##############################################################################

message(STATUS "\n\n========= Project Configuration ========\n")

set(BUILD_SHARED_LIBS true)

ament_python_install_package(${PROJECT_NAME})

include(${PROJECT_SOURCE_DIR}/../cmake/DefaultCFlags.cmake)


##############################################################################
# Python Setup
##############################################################################
# Pin down python 3.6
find_package(PythonInterp 3.6 REQUIRED)

# Unfortunately no easy way to get site-packages / dist-packages
set(PYTHON_INSTALL_DIR lib/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/site-packages)

##############################################################################
# Tests
##############################################################################


##############################################################################
# Sources
##############################################################################

install(
  PROGRAMS app/pybind_test.py
  DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
  )

# find_package(ament_cmake_clang_format REQUIRED)
# enable_testing()
# ament_clang_format(CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../.clang-format)


# ament_add_pytest_test(pybind_pytest
#   app/pybind_test.py
#   # Avoid pytest from importing the module stub
#   WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
# )


##############################################################################
# Export
##############################################################################

# install(
#   DIRECTORY include/
#   DESTINATION include
# )

# ament_export_include_directories(include)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(maliput)
# ament_export_interfaces(${PROJECT_NAME}-targets HAS_LIBRARY_TARGET)

ament_package()


