cmake_minimum_required(VERSION 2.8)
project(fastrapp)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})

find_package(OpenCV REQUIRED)
find_package(PCL 1.6 REQUIRED)
include(boost-python.cmake)

# http://cmake.3232098.n2.nabble.com/Default-value-for-CMAKE-BUILD-TYPE-td7550756.html
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Debug' as none was specified.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo")
endif()


set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(BUILD_SHARED_LIBS true)


option(OPT_ROS "build stuff involving ros, e.g. record_demo program" OFF)

if (OPT_ROS)
  set(ROS_VERSION "groovy" CACHE STRING "what version of ros")

  set(ROS_INCLUDE_DIRS "/opt/ros/${ROS_VERSION}/include")
  set(ROS_LIB_DIR "/opt/ros/${ROS_VERSION}/lib")
  set(ROS_LIBRARIES "${ROS_LIB_DIR}/libroscpp.so" "${ROS_LIB_DIR}/librostime.so" "${ROS_LIB_DIR}/librosconsole.so" "${ROS_LIB_DIR}/libroscpp_serialization.so")
endif (OPT_ROS)

include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${ROS_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_definitions("-DEIGEN_DEFAULT_TO_ROW_MAJOR")

if (OPT_ROS)
  add_executable(record_rgbd_video record_rgbd_video.cpp config.cpp)
  target_link_libraries(record_rgbd_video ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${ROS_LIBRARIES} boost_program_options)
endif(OPT_ROS)

include_directories(${PYTHON_NUMPY_INCLUDE_DIR})
boost_python_module(fastrapp fastrapp.cpp)



