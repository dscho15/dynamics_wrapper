cmake_minimum_required(VERSION 3.0.2)
project(dynamics_wrapper)

## Compile as C++20
set(CMAKE_CXX_STANDARD 17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roscpp_serialization
  std_msgs 
  kdl_parser
  eigen_conversions
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS python)
find_package(PythonLibs 3.7 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS NumPy)

## Orocos_kdl
find_package(orocos_kdl REQUIRED)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
        INCLUDE_DIRS 
            include
        LIBRARIES 
            kdl_interface
            _kdl_interface_cpp
        CATKIN_DEPENDS 
            roscpp
            eigen_conversions
            kdl_parser
        DEPENDS
            orocos_kdl
)

###########
## Build ##
###########

# include Boost and Python.
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS}
        ${orocos_kdl_INCLUDE_DIRS}
        ${Python3_INCLUDE_DIRS}
        ${Python3_NumPy_INCLUDE_DIRS}
        )

## Declare a cpp library
add_library(kdl_interface src/kdl_interface.cpp)
add_library(_kdl_interface_cpp src/_kdl_interface.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(kdl_interface ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})
target_link_libraries(_kdl_interface_cpp kdl_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${orocos_kdl_LIBRARIES} ${Python3_LIBRARIES})

# Don't prepend wrapper library name with lib and add to Python libs.
set_target_properties(_kdl_interface_cpp PROPERTIES PREFIX "" LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})