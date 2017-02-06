# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "dji_api: 9 messages, 0 services")

set(MSG_I_FLAGS "-Idji_api:/home/bobby/fyp/catkin_ws/src/dji_api/msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(dji_api_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_gps.msg" NAME_WE)
add_custom_target(_dji_api_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_api" "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_gps.msg" ""
)

get_filename_component(_filename "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_quaternion.msg" NAME_WE)
add_custom_target(_dji_api_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_api" "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_quaternion.msg" ""
)

get_filename_component(_filename "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_rc.msg" NAME_WE)
add_custom_target(_dji_api_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_api" "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_rc.msg" ""
)

get_filename_component(_filename "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_ctrl_data.msg" NAME_WE)
add_custom_target(_dji_api_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_api" "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_ctrl_data.msg" "geometry_msgs/Quaternion:geometry_msgs/QuaternionStamped:std_msgs/Header"
)

get_filename_component(_filename "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_gimbal.msg" NAME_WE)
add_custom_target(_dji_api_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_api" "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_gimbal.msg" ""
)

get_filename_component(_filename "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_accelerate.msg" NAME_WE)
add_custom_target(_dji_api_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_api" "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_accelerate.msg" ""
)

get_filename_component(_filename "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_velocity.msg" NAME_WE)
add_custom_target(_dji_api_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_api" "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_velocity.msg" ""
)

get_filename_component(_filename "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_magnetic.msg" NAME_WE)
add_custom_target(_dji_api_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_api" "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_magnetic.msg" ""
)

get_filename_component(_filename "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_wrench.msg" NAME_WE)
add_custom_target(_dji_api_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_api" "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_wrench.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(dji_api
  "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_gps.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_api
)
_generate_msg_cpp(dji_api
  "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_quaternion.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_api
)
_generate_msg_cpp(dji_api
  "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_rc.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_api
)
_generate_msg_cpp(dji_api
  "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_ctrl_data.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/QuaternionStamped.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_api
)
_generate_msg_cpp(dji_api
  "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_gimbal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_api
)
_generate_msg_cpp(dji_api
  "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_accelerate.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_api
)
_generate_msg_cpp(dji_api
  "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_velocity.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_api
)
_generate_msg_cpp(dji_api
  "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_magnetic.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_api
)
_generate_msg_cpp(dji_api
  "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_wrench.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_api
)

### Generating Services

### Generating Module File
_generate_module_cpp(dji_api
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_api
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(dji_api_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(dji_api_generate_messages dji_api_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_gps.msg" NAME_WE)
add_dependencies(dji_api_generate_messages_cpp _dji_api_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_quaternion.msg" NAME_WE)
add_dependencies(dji_api_generate_messages_cpp _dji_api_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_rc.msg" NAME_WE)
add_dependencies(dji_api_generate_messages_cpp _dji_api_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_ctrl_data.msg" NAME_WE)
add_dependencies(dji_api_generate_messages_cpp _dji_api_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_gimbal.msg" NAME_WE)
add_dependencies(dji_api_generate_messages_cpp _dji_api_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_accelerate.msg" NAME_WE)
add_dependencies(dji_api_generate_messages_cpp _dji_api_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_velocity.msg" NAME_WE)
add_dependencies(dji_api_generate_messages_cpp _dji_api_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_magnetic.msg" NAME_WE)
add_dependencies(dji_api_generate_messages_cpp _dji_api_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_wrench.msg" NAME_WE)
add_dependencies(dji_api_generate_messages_cpp _dji_api_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dji_api_gencpp)
add_dependencies(dji_api_gencpp dji_api_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dji_api_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(dji_api
  "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_gps.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_api
)
_generate_msg_lisp(dji_api
  "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_quaternion.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_api
)
_generate_msg_lisp(dji_api
  "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_rc.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_api
)
_generate_msg_lisp(dji_api
  "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_ctrl_data.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/QuaternionStamped.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_api
)
_generate_msg_lisp(dji_api
  "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_gimbal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_api
)
_generate_msg_lisp(dji_api
  "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_accelerate.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_api
)
_generate_msg_lisp(dji_api
  "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_velocity.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_api
)
_generate_msg_lisp(dji_api
  "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_magnetic.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_api
)
_generate_msg_lisp(dji_api
  "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_wrench.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_api
)

### Generating Services

### Generating Module File
_generate_module_lisp(dji_api
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_api
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(dji_api_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(dji_api_generate_messages dji_api_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_gps.msg" NAME_WE)
add_dependencies(dji_api_generate_messages_lisp _dji_api_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_quaternion.msg" NAME_WE)
add_dependencies(dji_api_generate_messages_lisp _dji_api_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_rc.msg" NAME_WE)
add_dependencies(dji_api_generate_messages_lisp _dji_api_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_ctrl_data.msg" NAME_WE)
add_dependencies(dji_api_generate_messages_lisp _dji_api_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_gimbal.msg" NAME_WE)
add_dependencies(dji_api_generate_messages_lisp _dji_api_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_accelerate.msg" NAME_WE)
add_dependencies(dji_api_generate_messages_lisp _dji_api_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_velocity.msg" NAME_WE)
add_dependencies(dji_api_generate_messages_lisp _dji_api_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_magnetic.msg" NAME_WE)
add_dependencies(dji_api_generate_messages_lisp _dji_api_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_wrench.msg" NAME_WE)
add_dependencies(dji_api_generate_messages_lisp _dji_api_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dji_api_genlisp)
add_dependencies(dji_api_genlisp dji_api_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dji_api_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(dji_api
  "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_gps.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_api
)
_generate_msg_py(dji_api
  "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_quaternion.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_api
)
_generate_msg_py(dji_api
  "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_rc.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_api
)
_generate_msg_py(dji_api
  "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_ctrl_data.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/QuaternionStamped.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_api
)
_generate_msg_py(dji_api
  "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_gimbal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_api
)
_generate_msg_py(dji_api
  "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_accelerate.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_api
)
_generate_msg_py(dji_api
  "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_velocity.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_api
)
_generate_msg_py(dji_api
  "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_magnetic.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_api
)
_generate_msg_py(dji_api
  "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_wrench.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_api
)

### Generating Services

### Generating Module File
_generate_module_py(dji_api
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_api
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(dji_api_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(dji_api_generate_messages dji_api_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_gps.msg" NAME_WE)
add_dependencies(dji_api_generate_messages_py _dji_api_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_quaternion.msg" NAME_WE)
add_dependencies(dji_api_generate_messages_py _dji_api_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_rc.msg" NAME_WE)
add_dependencies(dji_api_generate_messages_py _dji_api_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_ctrl_data.msg" NAME_WE)
add_dependencies(dji_api_generate_messages_py _dji_api_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_gimbal.msg" NAME_WE)
add_dependencies(dji_api_generate_messages_py _dji_api_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_accelerate.msg" NAME_WE)
add_dependencies(dji_api_generate_messages_py _dji_api_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_velocity.msg" NAME_WE)
add_dependencies(dji_api_generate_messages_py _dji_api_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_magnetic.msg" NAME_WE)
add_dependencies(dji_api_generate_messages_py _dji_api_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bobby/fyp/catkin_ws/src/dji_api/msg/api_wrench.msg" NAME_WE)
add_dependencies(dji_api_generate_messages_py _dji_api_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dji_api_genpy)
add_dependencies(dji_api_genpy dji_api_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dji_api_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_api)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_api
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(dji_api_generate_messages_cpp geometry_msgs_generate_messages_cpp)
add_dependencies(dji_api_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_api)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_api
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(dji_api_generate_messages_lisp geometry_msgs_generate_messages_lisp)
add_dependencies(dji_api_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_api)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_api\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_api
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(dji_api_generate_messages_py geometry_msgs_generate_messages_py)
add_dependencies(dji_api_generate_messages_py std_msgs_generate_messages_py)
