cmake_minimum_required(VERSION 2.8.3)
project(sonar_processing)
set(ROS_BUILD_TYPE Debug)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  avora_msgs
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  nav_msgs
  tf
  nodelet
  pluginlib
  tf
  type_definitions
)



## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV)
find_package(PCL REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   getNewScan.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   avora_msgs#   sensor_msgs#   std_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES SonarToCloud_nodelet Thresholder_nodelet OutlierRemover_nodelet Wall2DDetector_nodelet
  CATKIN_DEPENDS avora_msgs cv_bridge image_transport roscpp sensor_msgs std_msgs nav_msgs tf nodelet pluginlib type_definitions
 # LIBRARIES sonar_processing
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
	include
	${catkin_INCLUDE_DIRS}
)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(${PCL_INCLUDE_DIRS})

## Declare a cpp library
set(SONARTOCLOUD_SOURCES src/SonarToCloud.cpp include/SonarToCloud.h)
add_library(SonarToCloud_nodelet ${SONARTOCLOUD_SOURCES})

set(THRESHOLDER_SOURCES src/Thresholder.cpp include/Thresholder.h)
add_library(Thresholder_nodelet ${THRESHOLDER_SOURCES})

set(OUTLIER_REMOVER_SOURCES src/OutlierRemover.cpp include/OutlierRemover.h)
add_library(OutlierRemover_nodelet ${OUTLIER_REMOVER_SOURCES})

set(WALL2DDETECTOR_SOURCES src/Wall2DDetector.cpp include/Wall2DDetector.h)
add_library(Wall2DDetector_nodelet ${WALL2DDETECTOR_SOURCES})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes

## Specify libraries to link a library or executable target against
# target_link_libraries(sonar_processing_node
#   ${catkin_LIBRARIES}
# )

## Create the nodelet library
target_link_libraries(SonarToCloud_nodelet ${catkin_LIBRARIES})
target_link_libraries(Thresholder_nodelet ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(OutlierRemover_nodelet ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(Wall2DDetector_nodelet ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})


add_dependencies(SonarToCloud_nodelet avora_msgs_generate_messages_cpp)
add_dependencies(Thresholder_nodelet avora_msgs_generate_messages_cpp)
add_dependencies(OutlierRemover_nodelet avora_msgs_generate_messages_cpp)
add_dependencies(Wall2DDetector_nodelet avora_msgs_generate_messages_cpp)

if(catkin_EXPORTED_LIBRARIES)
	add_dependencies(SonarToCloud_nodelet ${catkin_EXPORTED_LIBRARIES})
	add_dependencies(Thresholder_nodelet ${catkin_EXPORTED_LIBRARIES})
        add_dependencies(OutlierRemover_nodelet ${catkin_EXPORTED_LIBRARIES})
        add_dependencies(Wall2DDetector_nodelet ${catkin_EXPORTED_LIBRARIES})
endif()




#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark the nodelet library for installations
install(TARGETS 
	SonarToCloud_nodelet
	Thresholder_nodelet
	OutlierRemover_nodelet
        Wall2DDetector_nodelet
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark executables and/or libraries for installation
# install(TARGETS sonar_processing sonar_processing_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )
install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
        PATTERN ".git" EXCLUDE)
## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
	SonarToCloud_nodelet_plugins.xml SonarToCloud.launch SonarToCloud_default.yaml
	Thresholder_nodelet_plugins.xml Thresholder.launch Thresholder_default.yaml
        OutlierRemover_nodelet_plugins.xml OutlierRemover.launch OutlierRemover_default.yaml
        Wall2DDetector_nodelet_plugins.xml Wall2DDetector.launch Wall2DDetector_default.yaml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_sonar_processing.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
