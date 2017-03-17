# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "bluefox2: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(bluefox2_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/ubuntu/fyp/catkin_ws/src/bluefox2/srv/SetExposeSrv.srv" NAME_WE)
add_custom_target(_bluefox2_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bluefox2" "/home/ubuntu/fyp/catkin_ws/src/bluefox2/srv/SetExposeSrv.srv" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(bluefox2
  "/home/ubuntu/fyp/catkin_ws/src/bluefox2/srv/SetExposeSrv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bluefox2
)

### Generating Module File
_generate_module_cpp(bluefox2
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bluefox2
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(bluefox2_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(bluefox2_generate_messages bluefox2_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/fyp/catkin_ws/src/bluefox2/srv/SetExposeSrv.srv" NAME_WE)
add_dependencies(bluefox2_generate_messages_cpp _bluefox2_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(bluefox2_gencpp)
add_dependencies(bluefox2_gencpp bluefox2_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS bluefox2_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(bluefox2
  "/home/ubuntu/fyp/catkin_ws/src/bluefox2/srv/SetExposeSrv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bluefox2
)

### Generating Module File
_generate_module_lisp(bluefox2
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bluefox2
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(bluefox2_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(bluefox2_generate_messages bluefox2_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/fyp/catkin_ws/src/bluefox2/srv/SetExposeSrv.srv" NAME_WE)
add_dependencies(bluefox2_generate_messages_lisp _bluefox2_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(bluefox2_genlisp)
add_dependencies(bluefox2_genlisp bluefox2_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS bluefox2_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(bluefox2
  "/home/ubuntu/fyp/catkin_ws/src/bluefox2/srv/SetExposeSrv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bluefox2
)

### Generating Module File
_generate_module_py(bluefox2
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bluefox2
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(bluefox2_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(bluefox2_generate_messages bluefox2_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/fyp/catkin_ws/src/bluefox2/srv/SetExposeSrv.srv" NAME_WE)
add_dependencies(bluefox2_generate_messages_py _bluefox2_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(bluefox2_genpy)
add_dependencies(bluefox2_genpy bluefox2_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS bluefox2_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bluefox2)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bluefox2
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(bluefox2_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bluefox2)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bluefox2
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(bluefox2_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bluefox2)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bluefox2\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bluefox2
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(bluefox2_generate_messages_py std_msgs_generate_messages_py)
