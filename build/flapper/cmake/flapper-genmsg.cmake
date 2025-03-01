# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "flapper: 2 messages, 0 services")

set(MSG_I_FLAGS "-Iflapper:/home/mech-user/flapper_ws/src/flapper/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(flapper_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/mech-user/flapper_ws/src/flapper/msg/Stabilizer.msg" NAME_WE)
add_custom_target(_flapper_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "flapper" "/home/mech-user/flapper_ws/src/flapper/msg/Stabilizer.msg" ""
)

get_filename_component(_filename "/home/mech-user/flapper_ws/src/flapper/msg/Rpm.msg" NAME_WE)
add_custom_target(_flapper_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "flapper" "/home/mech-user/flapper_ws/src/flapper/msg/Rpm.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(flapper
  "/home/mech-user/flapper_ws/src/flapper/msg/Stabilizer.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/flapper
)
_generate_msg_cpp(flapper
  "/home/mech-user/flapper_ws/src/flapper/msg/Rpm.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/flapper
)

### Generating Services

### Generating Module File
_generate_module_cpp(flapper
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/flapper
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(flapper_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(flapper_generate_messages flapper_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mech-user/flapper_ws/src/flapper/msg/Stabilizer.msg" NAME_WE)
add_dependencies(flapper_generate_messages_cpp _flapper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mech-user/flapper_ws/src/flapper/msg/Rpm.msg" NAME_WE)
add_dependencies(flapper_generate_messages_cpp _flapper_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(flapper_gencpp)
add_dependencies(flapper_gencpp flapper_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS flapper_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(flapper
  "/home/mech-user/flapper_ws/src/flapper/msg/Stabilizer.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/flapper
)
_generate_msg_eus(flapper
  "/home/mech-user/flapper_ws/src/flapper/msg/Rpm.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/flapper
)

### Generating Services

### Generating Module File
_generate_module_eus(flapper
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/flapper
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(flapper_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(flapper_generate_messages flapper_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mech-user/flapper_ws/src/flapper/msg/Stabilizer.msg" NAME_WE)
add_dependencies(flapper_generate_messages_eus _flapper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mech-user/flapper_ws/src/flapper/msg/Rpm.msg" NAME_WE)
add_dependencies(flapper_generate_messages_eus _flapper_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(flapper_geneus)
add_dependencies(flapper_geneus flapper_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS flapper_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(flapper
  "/home/mech-user/flapper_ws/src/flapper/msg/Stabilizer.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/flapper
)
_generate_msg_lisp(flapper
  "/home/mech-user/flapper_ws/src/flapper/msg/Rpm.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/flapper
)

### Generating Services

### Generating Module File
_generate_module_lisp(flapper
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/flapper
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(flapper_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(flapper_generate_messages flapper_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mech-user/flapper_ws/src/flapper/msg/Stabilizer.msg" NAME_WE)
add_dependencies(flapper_generate_messages_lisp _flapper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mech-user/flapper_ws/src/flapper/msg/Rpm.msg" NAME_WE)
add_dependencies(flapper_generate_messages_lisp _flapper_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(flapper_genlisp)
add_dependencies(flapper_genlisp flapper_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS flapper_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(flapper
  "/home/mech-user/flapper_ws/src/flapper/msg/Stabilizer.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/flapper
)
_generate_msg_nodejs(flapper
  "/home/mech-user/flapper_ws/src/flapper/msg/Rpm.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/flapper
)

### Generating Services

### Generating Module File
_generate_module_nodejs(flapper
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/flapper
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(flapper_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(flapper_generate_messages flapper_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mech-user/flapper_ws/src/flapper/msg/Stabilizer.msg" NAME_WE)
add_dependencies(flapper_generate_messages_nodejs _flapper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mech-user/flapper_ws/src/flapper/msg/Rpm.msg" NAME_WE)
add_dependencies(flapper_generate_messages_nodejs _flapper_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(flapper_gennodejs)
add_dependencies(flapper_gennodejs flapper_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS flapper_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(flapper
  "/home/mech-user/flapper_ws/src/flapper/msg/Stabilizer.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/flapper
)
_generate_msg_py(flapper
  "/home/mech-user/flapper_ws/src/flapper/msg/Rpm.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/flapper
)

### Generating Services

### Generating Module File
_generate_module_py(flapper
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/flapper
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(flapper_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(flapper_generate_messages flapper_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mech-user/flapper_ws/src/flapper/msg/Stabilizer.msg" NAME_WE)
add_dependencies(flapper_generate_messages_py _flapper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mech-user/flapper_ws/src/flapper/msg/Rpm.msg" NAME_WE)
add_dependencies(flapper_generate_messages_py _flapper_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(flapper_genpy)
add_dependencies(flapper_genpy flapper_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS flapper_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/flapper)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/flapper
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(flapper_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(flapper_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/flapper)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/flapper
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(flapper_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(flapper_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/flapper)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/flapper
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(flapper_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(flapper_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/flapper)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/flapper
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(flapper_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(flapper_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/flapper)
  install(CODE "execute_process(COMMAND \"/home/mech-user/.pyenv/shims/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/flapper\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/flapper
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(flapper_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(flapper_generate_messages_py geometry_msgs_generate_messages_py)
endif()
