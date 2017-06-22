cmake_minimum_required(VERSION 2.8.3)
project(dji_pid)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  geometry_msgs 
  sensor_msgs
  dji_sdk
)


SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(Eigen REQUIRED)

## find pid lib
file(GLOB MY_PID_LIB_SOURCES ${PROJECT_SOURCE_DIR}/include/PID_lib/*.cpp)
include_directories("${PROJECT_SOURCE_DIR}/include/PID_lib")

## find EKF lib
#file(GLOB MY_EKF_LIB_SOURCES ${PROJECT_SOURCE_DIR}/include/estimate_lib/*.cpp)
#include_directories("${PROJECT_SOURCE_DIR}/include/EKF_lib")

## find Yu Yun math library
file(GLOB UAV_MATH_LIB_SOURCES include/math_lib/*.c)
include_directories("${PROJECT_SOURCE_DIR}/include/math_lib")
SET_SOURCE_FILES_PROPERTIES(${UAV_MATH_LIB_SOURCES} PROPERTIES LANGUAGE CXX)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

INCLUDE_DIRECTORIES(
   include
   ${Eigen_INCLUDE_DIRS}
   ${OpenCv_INCLUDE_DIRS}
)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   pid_ctrl_data.msg
   move_target.msg
 )

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
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
  geometry_msgs 
 )

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
#  INCLUDE_DIRS include
#  LIBRARIES dji_pid
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(dji_pid
#   src/${PROJECT_NAME}/dji_pid.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(dji_pid ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(pid_ctrl_node src/pid_controller.cpp ${MY_PID_LIB_SOURCES})
#add_executable(ekf_node src/ekf_vicon.cpp ${UAV_MATH_LIB_SOURCES} ${MY_EKF_LIB_SOURCES})
#add_executable(ekf_test src/ekf_test.cpp ${MY_EKF_LIB_SOURCES})

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(dji_pid_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(pid_ctrl_node ${catkin_LIBRARIES} )
#target_link_libraries(ekf_node ${catkin_LIBRARIES} )


