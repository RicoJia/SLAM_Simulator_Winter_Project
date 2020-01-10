# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "tsim: 1 messages, 1 services")

set(MSG_I_FLAGS "-Itsim:/home/ricojia/main-assignment-RicoJia/src/tsim/msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(tsim_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/ricojia/main-assignment-RicoJia/src/tsim/srv/traj_reset.srv" NAME_WE)
add_custom_target(_tsim_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tsim" "/home/ricojia/main-assignment-RicoJia/src/tsim/srv/traj_reset.srv" ""
)

get_filename_component(_filename "/home/ricojia/main-assignment-RicoJia/src/tsim/msg/PoseError.msg" NAME_WE)
add_custom_target(_tsim_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tsim" "/home/ricojia/main-assignment-RicoJia/src/tsim/msg/PoseError.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(tsim
  "/home/ricojia/main-assignment-RicoJia/src/tsim/msg/PoseError.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tsim
)

### Generating Services
_generate_srv_cpp(tsim
  "/home/ricojia/main-assignment-RicoJia/src/tsim/srv/traj_reset.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tsim
)

### Generating Module File
_generate_module_cpp(tsim
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tsim
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(tsim_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(tsim_generate_messages tsim_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ricojia/main-assignment-RicoJia/src/tsim/srv/traj_reset.srv" NAME_WE)
add_dependencies(tsim_generate_messages_cpp _tsim_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ricojia/main-assignment-RicoJia/src/tsim/msg/PoseError.msg" NAME_WE)
add_dependencies(tsim_generate_messages_cpp _tsim_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tsim_gencpp)
add_dependencies(tsim_gencpp tsim_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tsim_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(tsim
  "/home/ricojia/main-assignment-RicoJia/src/tsim/msg/PoseError.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tsim
)

### Generating Services
_generate_srv_eus(tsim
  "/home/ricojia/main-assignment-RicoJia/src/tsim/srv/traj_reset.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tsim
)

### Generating Module File
_generate_module_eus(tsim
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tsim
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(tsim_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(tsim_generate_messages tsim_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ricojia/main-assignment-RicoJia/src/tsim/srv/traj_reset.srv" NAME_WE)
add_dependencies(tsim_generate_messages_eus _tsim_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ricojia/main-assignment-RicoJia/src/tsim/msg/PoseError.msg" NAME_WE)
add_dependencies(tsim_generate_messages_eus _tsim_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tsim_geneus)
add_dependencies(tsim_geneus tsim_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tsim_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(tsim
  "/home/ricojia/main-assignment-RicoJia/src/tsim/msg/PoseError.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tsim
)

### Generating Services
_generate_srv_lisp(tsim
  "/home/ricojia/main-assignment-RicoJia/src/tsim/srv/traj_reset.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tsim
)

### Generating Module File
_generate_module_lisp(tsim
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tsim
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(tsim_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(tsim_generate_messages tsim_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ricojia/main-assignment-RicoJia/src/tsim/srv/traj_reset.srv" NAME_WE)
add_dependencies(tsim_generate_messages_lisp _tsim_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ricojia/main-assignment-RicoJia/src/tsim/msg/PoseError.msg" NAME_WE)
add_dependencies(tsim_generate_messages_lisp _tsim_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tsim_genlisp)
add_dependencies(tsim_genlisp tsim_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tsim_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(tsim
  "/home/ricojia/main-assignment-RicoJia/src/tsim/msg/PoseError.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tsim
)

### Generating Services
_generate_srv_nodejs(tsim
  "/home/ricojia/main-assignment-RicoJia/src/tsim/srv/traj_reset.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tsim
)

### Generating Module File
_generate_module_nodejs(tsim
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tsim
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(tsim_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(tsim_generate_messages tsim_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ricojia/main-assignment-RicoJia/src/tsim/srv/traj_reset.srv" NAME_WE)
add_dependencies(tsim_generate_messages_nodejs _tsim_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ricojia/main-assignment-RicoJia/src/tsim/msg/PoseError.msg" NAME_WE)
add_dependencies(tsim_generate_messages_nodejs _tsim_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tsim_gennodejs)
add_dependencies(tsim_gennodejs tsim_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tsim_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(tsim
  "/home/ricojia/main-assignment-RicoJia/src/tsim/msg/PoseError.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tsim
)

### Generating Services
_generate_srv_py(tsim
  "/home/ricojia/main-assignment-RicoJia/src/tsim/srv/traj_reset.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tsim
)

### Generating Module File
_generate_module_py(tsim
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tsim
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(tsim_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(tsim_generate_messages tsim_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ricojia/main-assignment-RicoJia/src/tsim/srv/traj_reset.srv" NAME_WE)
add_dependencies(tsim_generate_messages_py _tsim_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ricojia/main-assignment-RicoJia/src/tsim/msg/PoseError.msg" NAME_WE)
add_dependencies(tsim_generate_messages_py _tsim_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tsim_genpy)
add_dependencies(tsim_genpy tsim_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tsim_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tsim)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tsim
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_srvs_generate_messages_cpp)
  add_dependencies(tsim_generate_messages_cpp std_srvs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tsim)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tsim
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_srvs_generate_messages_eus)
  add_dependencies(tsim_generate_messages_eus std_srvs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tsim)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tsim
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_srvs_generate_messages_lisp)
  add_dependencies(tsim_generate_messages_lisp std_srvs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tsim)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tsim
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_srvs_generate_messages_nodejs)
  add_dependencies(tsim_generate_messages_nodejs std_srvs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tsim)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tsim\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tsim
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_srvs_generate_messages_py)
  add_dependencies(tsim_generate_messages_py std_srvs_generate_messages_py)
endif()
