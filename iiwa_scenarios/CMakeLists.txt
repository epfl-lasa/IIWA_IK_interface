cmake_minimum_required(VERSION 2.8.3)
project(iiwa_scenarios)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  mathlib
  robotlib
  skinematics
  tf
  kuka_fri_bridge
)

set(CMAKE_CXX_FLAGS "-std=c++11")


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
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
#   Service1.srv
#   Service2.srv
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
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
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
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS iiwa_ik/include
#  LIBRARIES transition_scenarios
#  CATKIN_DEPENDS mathlib robotlib
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
 include_directories(iiwa_ik/include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

FILE(GLOB theSourceFiles iiwa_ik/src/*.cpp)
 add_library(iiwa_ik  ${theSourceFiles})


 target_link_libraries(iiwa_ik
  ${catkin_LIBRARIES} ${OpenMP_FLAGS}
 )

###################################
## RobotToolKit specific stuff  ###
###################################
# to comply with the file structure of RobotToolKit, you need to create symlinks 
# to any compiled modules in the RobotToolKit/module folder. This is automated below.
# just add the modules of this package as set(rtk_modules myAwesomeModule1 myAwesomeModule2)
# modules can be world or robot modules. Note that you need to specify your targets using 
# standard cmake/catkin procedures above. This is step is only for linking the build modules.
set(rtk_modules iiwa_ik)
execute_process(COMMAND rospack find rtk_pkg_tools OUTPUT_VARIABLE rtk_pkg_tools_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)
include(${rtk_pkg_tools_DIR}/rtk_create_symlinks.cmake)

# Uncomment the following line if you want catkin dependencies you specify in CMakeLists.txt to be automatically copied to the package.xml
execute_process(COMMAND ${rtk_pkg_tools_DIR}/scripts/CopyDepsFromCMakeListsToPackageXML.py ${PROJECT_NAME})

