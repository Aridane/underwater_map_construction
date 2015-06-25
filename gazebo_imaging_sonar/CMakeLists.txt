cmake_minimum_required(VERSION 2.8.3)
project(gazebo_imaging_sonar)

FIND_PACKAGE( Boost 1.40 COMPONENTS system REQUIRED )
set (CMAKE_CXX_FLAGS "-g -Wall")

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(SDF sdformat)
endif()

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/msgs
  )
link_directories(${GAZEBO_LIBRARY_DIRS} ${SDF_LIBRARY_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/msgs)
add_subdirectory(msgs)

set(IMAGING_SONAR_SOURCES src/ImagingSonar.cc include/ImagingSonar.hh)

add_library(gazebo_imaging_sonar ${IMAGING_SONAR_SOURCES})
target_link_libraries(gazebo_imaging_sonar gazebo_imaging_sonar_msgs ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES}) 
add_dependencies(gazebo_imaging_sonar gazebo_imaging_sonar_msgs)

