cmake_minimum_required(VERSION 2.8.3)
project(threading_gustavo)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED roscpp rospy actionlib actionlib_msgs geometry_msgs yumi_experiments)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
 add_action_files(
   FILES RunFolding.action ApproachController.action AdmittanceController.action SensorCalibration.action
 )

catkin_python_setup()

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   actionlib_msgs geometry_msgs # Or other packages containing msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
#  INCLUDE_DIRS include
#  LIBRARIES threading
  CATKIN_DEPENDS roscpp rospy actionlib actionlib_msgs geometry_msgs yumi_experiments
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
 ${catkin_INCLUDE_DIRS}
)

add_executable(gustavo_teste src/gustavo_teste.cpp)
target_link_libraries(   gustavo_teste   ${catkin_LIBRARIES} )
add_dependencies(gustavo_teste ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(joints_subscriber src/joints_subscriber.cpp)
target_link_libraries(joints_subscriber   ${catkin_LIBRARIES})
add_dependencies(joints_subscriber ${${PROJECT_NAME}_EXPORTED_TARGETS})

catkin_install_python(PROGRAMS scripts/gustavo_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
