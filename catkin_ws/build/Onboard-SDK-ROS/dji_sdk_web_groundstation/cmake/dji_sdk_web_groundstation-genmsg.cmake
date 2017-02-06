# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "dji_sdk_web_groundstation: 8 messages, 0 services")

set(MSG_I_FLAGS "-Idji_sdk_web_groundstation:/home/bobby/fyp/catkin_ws/src/Onboard-SDK-ROS/dji_sdk_web_groundstation/msg;-Idji_sdk_web_groundstation:/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/indigo/share/actionlib_msgs/cmake/../msg;-Idji_sdk:/home/bobby/fyp/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg;-Idji_sdk:/home/bobby/fyp/catkin_ws/devel/share/dji_sdk/msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg;-Inav_msgs:/opt/ros/indigo/share/nav_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(dji_sdk_web_groundstation_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/bobby/fyp/catkin_ws/src/Onboard-SDK-ROS/dji_sdk_web_groundstation/msg/MapNavSrvCmd.msg" NAME_WE)
add_custom_target(_dji_sdk_web_groundstation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_sdk_web_groundstation" "/home/bobby/fyp/catkin_ws/src/Onboard-SDK-ROS/dji_sdk_web_groundstation/msg/MapNavSrvCmd.msg" ""
)

get_filename_component(_filename "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveFeedback.msg" NAME_WE)
add_custom_target(_dji_sdk_web_groundstation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_sdk_web_groundstation" "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveFeedback.msg" ""
)

get_filename_component(_filename "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveActionResult.msg" NAME_WE)
add_custom_target(_dji_sdk_web_groundstation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_sdk_web_groundstation" "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveActionResult.msg" "actionlib_msgs/GoalStatus:actionlib_msgs/GoalID:dji_sdk_web_groundstation/WebWaypointReceiveResult:std_msgs/Header"
)

get_filename_component(_filename "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveActionFeedback.msg" NAME_WE)
add_custom_target(_dji_sdk_web_groundstation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_sdk_web_groundstation" "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveActionFeedback.msg" "actionlib_msgs/GoalStatus:dji_sdk_web_groundstation/WebWaypointReceiveFeedback:actionlib_msgs/GoalID:std_msgs/Header"
)

get_filename_component(_filename "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveGoal.msg" NAME_WE)
add_custom_target(_dji_sdk_web_groundstation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_sdk_web_groundstation" "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveGoal.msg" "dji_sdk/WaypointList:dji_sdk/Waypoint"
)

get_filename_component(_filename "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveAction.msg" NAME_WE)
add_custom_target(_dji_sdk_web_groundstation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_sdk_web_groundstation" "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveAction.msg" "dji_sdk/WaypointList:dji_sdk_web_groundstation/WebWaypointReceiveResult:actionlib_msgs/GoalStatus:actionlib_msgs/GoalID:dji_sdk/Waypoint:dji_sdk_web_groundstation/WebWaypointReceiveFeedback:dji_sdk_web_groundstation/WebWaypointReceiveActionFeedback:std_msgs/Header:dji_sdk_web_groundstation/WebWaypointReceiveActionGoal:dji_sdk_web_groundstation/WebWaypointReceiveGoal:dji_sdk_web_groundstation/WebWaypointReceiveActionResult"
)

get_filename_component(_filename "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveResult.msg" NAME_WE)
add_custom_target(_dji_sdk_web_groundstation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_sdk_web_groundstation" "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveResult.msg" ""
)

get_filename_component(_filename "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveActionGoal.msg" NAME_WE)
add_custom_target(_dji_sdk_web_groundstation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_sdk_web_groundstation" "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveActionGoal.msg" "dji_sdk/WaypointList:actionlib_msgs/GoalID:dji_sdk_web_groundstation/WebWaypointReceiveGoal:std_msgs/Header:dji_sdk/Waypoint"
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(dji_sdk_web_groundstation
  "/home/bobby/fyp/catkin_ws/src/Onboard-SDK-ROS/dji_sdk_web_groundstation/msg/MapNavSrvCmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_sdk_web_groundstation
)
_generate_msg_cpp(dji_sdk_web_groundstation
  "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_sdk_web_groundstation
)
_generate_msg_cpp(dji_sdk_web_groundstation
  "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveResult.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_sdk_web_groundstation
)
_generate_msg_cpp(dji_sdk_web_groundstation
  "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_sdk_web_groundstation
)
_generate_msg_cpp(dji_sdk_web_groundstation
  "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/bobby/fyp/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/WaypointList.msg;/home/bobby/fyp/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_sdk_web_groundstation
)
_generate_msg_cpp(dji_sdk_web_groundstation
  "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveAction.msg"
  "${MSG_I_FLAGS}"
  "/home/bobby/fyp/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/WaypointList.msg;/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveResult.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/bobby/fyp/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/Waypoint.msg;/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveFeedback.msg;/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveActionFeedback.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveActionGoal.msg;/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveGoal.msg;/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveActionResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_sdk_web_groundstation
)
_generate_msg_cpp(dji_sdk_web_groundstation
  "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/bobby/fyp/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/WaypointList.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveGoal.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/bobby/fyp/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_sdk_web_groundstation
)
_generate_msg_cpp(dji_sdk_web_groundstation
  "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveFeedback.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_sdk_web_groundstation
)

### Generating Services

### Generating Module File
_generate_module_cpp(dji_sdk_web_groundstation
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_sdk_web_groundstation
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(dji_sdk_web_groundstation_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(dji_sdk_web_groundstation_generate_messages dji_sdk_web_groundstation_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bobby/fyp/catkin_ws/src/Onboard-SDK-ROS/dji_sdk_web_groundstation/msg/MapNavSrvCmd.msg" NAME_WE)
add_dependencies(dji_sdk_web_groundstation_generate_messages_cpp _dji_sdk_web_groundstation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveFeedback.msg" NAME_WE)
add_dependencies(dji_sdk_web_groundstation_generate_messages_cpp _dji_sdk_web_groundstation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveActionResult.msg" NAME_WE)
add_dependencies(dji_sdk_web_groundstation_generate_messages_cpp _dji_sdk_web_groundstation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveActionFeedback.msg" NAME_WE)
add_dependencies(dji_sdk_web_groundstation_generate_messages_cpp _dji_sdk_web_groundstation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveGoal.msg" NAME_WE)
add_dependencies(dji_sdk_web_groundstation_generate_messages_cpp _dji_sdk_web_groundstation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveAction.msg" NAME_WE)
add_dependencies(dji_sdk_web_groundstation_generate_messages_cpp _dji_sdk_web_groundstation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveResult.msg" NAME_WE)
add_dependencies(dji_sdk_web_groundstation_generate_messages_cpp _dji_sdk_web_groundstation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveActionGoal.msg" NAME_WE)
add_dependencies(dji_sdk_web_groundstation_generate_messages_cpp _dji_sdk_web_groundstation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dji_sdk_web_groundstation_gencpp)
add_dependencies(dji_sdk_web_groundstation_gencpp dji_sdk_web_groundstation_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dji_sdk_web_groundstation_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(dji_sdk_web_groundstation
  "/home/bobby/fyp/catkin_ws/src/Onboard-SDK-ROS/dji_sdk_web_groundstation/msg/MapNavSrvCmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_sdk_web_groundstation
)
_generate_msg_lisp(dji_sdk_web_groundstation
  "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_sdk_web_groundstation
)
_generate_msg_lisp(dji_sdk_web_groundstation
  "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveResult.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_sdk_web_groundstation
)
_generate_msg_lisp(dji_sdk_web_groundstation
  "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_sdk_web_groundstation
)
_generate_msg_lisp(dji_sdk_web_groundstation
  "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/bobby/fyp/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/WaypointList.msg;/home/bobby/fyp/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_sdk_web_groundstation
)
_generate_msg_lisp(dji_sdk_web_groundstation
  "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveAction.msg"
  "${MSG_I_FLAGS}"
  "/home/bobby/fyp/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/WaypointList.msg;/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveResult.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/bobby/fyp/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/Waypoint.msg;/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveFeedback.msg;/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveActionFeedback.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveActionGoal.msg;/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveGoal.msg;/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveActionResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_sdk_web_groundstation
)
_generate_msg_lisp(dji_sdk_web_groundstation
  "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/bobby/fyp/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/WaypointList.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveGoal.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/bobby/fyp/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_sdk_web_groundstation
)
_generate_msg_lisp(dji_sdk_web_groundstation
  "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveFeedback.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_sdk_web_groundstation
)

### Generating Services

### Generating Module File
_generate_module_lisp(dji_sdk_web_groundstation
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_sdk_web_groundstation
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(dji_sdk_web_groundstation_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(dji_sdk_web_groundstation_generate_messages dji_sdk_web_groundstation_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bobby/fyp/catkin_ws/src/Onboard-SDK-ROS/dji_sdk_web_groundstation/msg/MapNavSrvCmd.msg" NAME_WE)
add_dependencies(dji_sdk_web_groundstation_generate_messages_lisp _dji_sdk_web_groundstation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveFeedback.msg" NAME_WE)
add_dependencies(dji_sdk_web_groundstation_generate_messages_lisp _dji_sdk_web_groundstation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveActionResult.msg" NAME_WE)
add_dependencies(dji_sdk_web_groundstation_generate_messages_lisp _dji_sdk_web_groundstation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveActionFeedback.msg" NAME_WE)
add_dependencies(dji_sdk_web_groundstation_generate_messages_lisp _dji_sdk_web_groundstation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveGoal.msg" NAME_WE)
add_dependencies(dji_sdk_web_groundstation_generate_messages_lisp _dji_sdk_web_groundstation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveAction.msg" NAME_WE)
add_dependencies(dji_sdk_web_groundstation_generate_messages_lisp _dji_sdk_web_groundstation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveResult.msg" NAME_WE)
add_dependencies(dji_sdk_web_groundstation_generate_messages_lisp _dji_sdk_web_groundstation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveActionGoal.msg" NAME_WE)
add_dependencies(dji_sdk_web_groundstation_generate_messages_lisp _dji_sdk_web_groundstation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dji_sdk_web_groundstation_genlisp)
add_dependencies(dji_sdk_web_groundstation_genlisp dji_sdk_web_groundstation_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dji_sdk_web_groundstation_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(dji_sdk_web_groundstation
  "/home/bobby/fyp/catkin_ws/src/Onboard-SDK-ROS/dji_sdk_web_groundstation/msg/MapNavSrvCmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk_web_groundstation
)
_generate_msg_py(dji_sdk_web_groundstation
  "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk_web_groundstation
)
_generate_msg_py(dji_sdk_web_groundstation
  "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveResult.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk_web_groundstation
)
_generate_msg_py(dji_sdk_web_groundstation
  "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk_web_groundstation
)
_generate_msg_py(dji_sdk_web_groundstation
  "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/bobby/fyp/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/WaypointList.msg;/home/bobby/fyp/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk_web_groundstation
)
_generate_msg_py(dji_sdk_web_groundstation
  "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveAction.msg"
  "${MSG_I_FLAGS}"
  "/home/bobby/fyp/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/WaypointList.msg;/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveResult.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/bobby/fyp/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/Waypoint.msg;/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveFeedback.msg;/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveActionFeedback.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveActionGoal.msg;/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveGoal.msg;/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveActionResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk_web_groundstation
)
_generate_msg_py(dji_sdk_web_groundstation
  "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/bobby/fyp/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/WaypointList.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveGoal.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/bobby/fyp/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk_web_groundstation
)
_generate_msg_py(dji_sdk_web_groundstation
  "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveFeedback.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk_web_groundstation
)

### Generating Services

### Generating Module File
_generate_module_py(dji_sdk_web_groundstation
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk_web_groundstation
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(dji_sdk_web_groundstation_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(dji_sdk_web_groundstation_generate_messages dji_sdk_web_groundstation_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bobby/fyp/catkin_ws/src/Onboard-SDK-ROS/dji_sdk_web_groundstation/msg/MapNavSrvCmd.msg" NAME_WE)
add_dependencies(dji_sdk_web_groundstation_generate_messages_py _dji_sdk_web_groundstation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveFeedback.msg" NAME_WE)
add_dependencies(dji_sdk_web_groundstation_generate_messages_py _dji_sdk_web_groundstation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveActionResult.msg" NAME_WE)
add_dependencies(dji_sdk_web_groundstation_generate_messages_py _dji_sdk_web_groundstation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveActionFeedback.msg" NAME_WE)
add_dependencies(dji_sdk_web_groundstation_generate_messages_py _dji_sdk_web_groundstation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveGoal.msg" NAME_WE)
add_dependencies(dji_sdk_web_groundstation_generate_messages_py _dji_sdk_web_groundstation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveAction.msg" NAME_WE)
add_dependencies(dji_sdk_web_groundstation_generate_messages_py _dji_sdk_web_groundstation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveResult.msg" NAME_WE)
add_dependencies(dji_sdk_web_groundstation_generate_messages_py _dji_sdk_web_groundstation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bobby/fyp/catkin_ws/devel/share/dji_sdk_web_groundstation/msg/WebWaypointReceiveActionGoal.msg" NAME_WE)
add_dependencies(dji_sdk_web_groundstation_generate_messages_py _dji_sdk_web_groundstation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dji_sdk_web_groundstation_genpy)
add_dependencies(dji_sdk_web_groundstation_genpy dji_sdk_web_groundstation_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dji_sdk_web_groundstation_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_sdk_web_groundstation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_sdk_web_groundstation
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(dji_sdk_web_groundstation_generate_messages_cpp std_msgs_generate_messages_cpp)
add_dependencies(dji_sdk_web_groundstation_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
add_dependencies(dji_sdk_web_groundstation_generate_messages_cpp dji_sdk_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_sdk_web_groundstation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_sdk_web_groundstation
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(dji_sdk_web_groundstation_generate_messages_lisp std_msgs_generate_messages_lisp)
add_dependencies(dji_sdk_web_groundstation_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
add_dependencies(dji_sdk_web_groundstation_generate_messages_lisp dji_sdk_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk_web_groundstation)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk_web_groundstation\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_sdk_web_groundstation
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(dji_sdk_web_groundstation_generate_messages_py std_msgs_generate_messages_py)
add_dependencies(dji_sdk_web_groundstation_generate_messages_py actionlib_msgs_generate_messages_py)
add_dependencies(dji_sdk_web_groundstation_generate_messages_py dji_sdk_generate_messages_py)
