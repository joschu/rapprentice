cmake_minimum_required(VERSION 2.8)
project(rgbd_video)

find_package(OpenCV REQUIRED)
find_package(PCL 1.6 REQUIRED)

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

set(ROS_INCLUDE_DIRS "/opt/ros/fuerte/include")
set(ROS_LIBRARIES roscpp)

include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${ROS_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS} "/opt/ros/fuerte/lib")
add_definitions(${PCL_DEFINITIONS})

add_executable(record_rgbd_video record_rgbd_video.cpp config.cpp)
target_link_libraries(record_rgbd_video ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} pcl_io roscpp rostime boost_program_options)
