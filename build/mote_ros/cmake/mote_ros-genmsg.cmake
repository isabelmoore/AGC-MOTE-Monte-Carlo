# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "mote_ros: 3 messages, 0 services")

set(MSG_I_FLAGS "-Imote_ros:/home/wizard/AGC-MOTE-Monte-Carlo/src/mote_ros/msg;-Istd_msgs:/home/wizard/miniconda3/envs/mote_ros/share/std_msgs/cmake/../msg;-Igeometry_msgs:/home/wizard/miniconda3/envs/mote_ros/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/home/wizard/miniconda3/envs/mote_ros/share/sensor_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(mote_ros_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/wizard/AGC-MOTE-Monte-Carlo/src/mote_ros/msg/State.msg" NAME_WE)
add_custom_target(_mote_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mote_ros" "/home/wizard/AGC-MOTE-Monte-Carlo/src/mote_ros/msg/State.msg" ""
)

get_filename_component(_filename "/home/wizard/AGC-MOTE-Monte-Carlo/src/mote_ros/msg/Yaw.msg" NAME_WE)
add_custom_target(_mote_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mote_ros" "/home/wizard/AGC-MOTE-Monte-Carlo/src/mote_ros/msg/Yaw.msg" ""
)

get_filename_component(_filename "/home/wizard/AGC-MOTE-Monte-Carlo/src/mote_ros/msg/Pos.msg" NAME_WE)
add_custom_target(_mote_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mote_ros" "/home/wizard/AGC-MOTE-Monte-Carlo/src/mote_ros/msg/Pos.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(mote_ros
  "/home/wizard/AGC-MOTE-Monte-Carlo/src/mote_ros/msg/State.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mote_ros
)
_generate_msg_cpp(mote_ros
  "/home/wizard/AGC-MOTE-Monte-Carlo/src/mote_ros/msg/Yaw.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mote_ros
)
_generate_msg_cpp(mote_ros
  "/home/wizard/AGC-MOTE-Monte-Carlo/src/mote_ros/msg/Pos.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mote_ros
)

### Generating Services

### Generating Module File
_generate_module_cpp(mote_ros
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mote_ros
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(mote_ros_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(mote_ros_generate_messages mote_ros_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wizard/AGC-MOTE-Monte-Carlo/src/mote_ros/msg/State.msg" NAME_WE)
add_dependencies(mote_ros_generate_messages_cpp _mote_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wizard/AGC-MOTE-Monte-Carlo/src/mote_ros/msg/Yaw.msg" NAME_WE)
add_dependencies(mote_ros_generate_messages_cpp _mote_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wizard/AGC-MOTE-Monte-Carlo/src/mote_ros/msg/Pos.msg" NAME_WE)
add_dependencies(mote_ros_generate_messages_cpp _mote_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mote_ros_gencpp)
add_dependencies(mote_ros_gencpp mote_ros_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mote_ros_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(mote_ros
  "/home/wizard/AGC-MOTE-Monte-Carlo/src/mote_ros/msg/State.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mote_ros
)
_generate_msg_eus(mote_ros
  "/home/wizard/AGC-MOTE-Monte-Carlo/src/mote_ros/msg/Yaw.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mote_ros
)
_generate_msg_eus(mote_ros
  "/home/wizard/AGC-MOTE-Monte-Carlo/src/mote_ros/msg/Pos.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mote_ros
)

### Generating Services

### Generating Module File
_generate_module_eus(mote_ros
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mote_ros
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(mote_ros_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(mote_ros_generate_messages mote_ros_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wizard/AGC-MOTE-Monte-Carlo/src/mote_ros/msg/State.msg" NAME_WE)
add_dependencies(mote_ros_generate_messages_eus _mote_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wizard/AGC-MOTE-Monte-Carlo/src/mote_ros/msg/Yaw.msg" NAME_WE)
add_dependencies(mote_ros_generate_messages_eus _mote_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wizard/AGC-MOTE-Monte-Carlo/src/mote_ros/msg/Pos.msg" NAME_WE)
add_dependencies(mote_ros_generate_messages_eus _mote_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mote_ros_geneus)
add_dependencies(mote_ros_geneus mote_ros_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mote_ros_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(mote_ros
  "/home/wizard/AGC-MOTE-Monte-Carlo/src/mote_ros/msg/State.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mote_ros
)
_generate_msg_lisp(mote_ros
  "/home/wizard/AGC-MOTE-Monte-Carlo/src/mote_ros/msg/Yaw.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mote_ros
)
_generate_msg_lisp(mote_ros
  "/home/wizard/AGC-MOTE-Monte-Carlo/src/mote_ros/msg/Pos.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mote_ros
)

### Generating Services

### Generating Module File
_generate_module_lisp(mote_ros
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mote_ros
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(mote_ros_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(mote_ros_generate_messages mote_ros_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wizard/AGC-MOTE-Monte-Carlo/src/mote_ros/msg/State.msg" NAME_WE)
add_dependencies(mote_ros_generate_messages_lisp _mote_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wizard/AGC-MOTE-Monte-Carlo/src/mote_ros/msg/Yaw.msg" NAME_WE)
add_dependencies(mote_ros_generate_messages_lisp _mote_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wizard/AGC-MOTE-Monte-Carlo/src/mote_ros/msg/Pos.msg" NAME_WE)
add_dependencies(mote_ros_generate_messages_lisp _mote_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mote_ros_genlisp)
add_dependencies(mote_ros_genlisp mote_ros_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mote_ros_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(mote_ros
  "/home/wizard/AGC-MOTE-Monte-Carlo/src/mote_ros/msg/State.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mote_ros
)
_generate_msg_nodejs(mote_ros
  "/home/wizard/AGC-MOTE-Monte-Carlo/src/mote_ros/msg/Yaw.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mote_ros
)
_generate_msg_nodejs(mote_ros
  "/home/wizard/AGC-MOTE-Monte-Carlo/src/mote_ros/msg/Pos.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mote_ros
)

### Generating Services

### Generating Module File
_generate_module_nodejs(mote_ros
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mote_ros
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(mote_ros_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(mote_ros_generate_messages mote_ros_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wizard/AGC-MOTE-Monte-Carlo/src/mote_ros/msg/State.msg" NAME_WE)
add_dependencies(mote_ros_generate_messages_nodejs _mote_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wizard/AGC-MOTE-Monte-Carlo/src/mote_ros/msg/Yaw.msg" NAME_WE)
add_dependencies(mote_ros_generate_messages_nodejs _mote_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wizard/AGC-MOTE-Monte-Carlo/src/mote_ros/msg/Pos.msg" NAME_WE)
add_dependencies(mote_ros_generate_messages_nodejs _mote_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mote_ros_gennodejs)
add_dependencies(mote_ros_gennodejs mote_ros_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mote_ros_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(mote_ros
  "/home/wizard/AGC-MOTE-Monte-Carlo/src/mote_ros/msg/State.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mote_ros
)
_generate_msg_py(mote_ros
  "/home/wizard/AGC-MOTE-Monte-Carlo/src/mote_ros/msg/Yaw.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mote_ros
)
_generate_msg_py(mote_ros
  "/home/wizard/AGC-MOTE-Monte-Carlo/src/mote_ros/msg/Pos.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mote_ros
)

### Generating Services

### Generating Module File
_generate_module_py(mote_ros
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mote_ros
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(mote_ros_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(mote_ros_generate_messages mote_ros_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wizard/AGC-MOTE-Monte-Carlo/src/mote_ros/msg/State.msg" NAME_WE)
add_dependencies(mote_ros_generate_messages_py _mote_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wizard/AGC-MOTE-Monte-Carlo/src/mote_ros/msg/Yaw.msg" NAME_WE)
add_dependencies(mote_ros_generate_messages_py _mote_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wizard/AGC-MOTE-Monte-Carlo/src/mote_ros/msg/Pos.msg" NAME_WE)
add_dependencies(mote_ros_generate_messages_py _mote_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mote_ros_genpy)
add_dependencies(mote_ros_genpy mote_ros_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mote_ros_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mote_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mote_ros
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(mote_ros_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(mote_ros_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(mote_ros_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mote_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mote_ros
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(mote_ros_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(mote_ros_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(mote_ros_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mote_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mote_ros
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(mote_ros_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(mote_ros_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(mote_ros_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mote_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mote_ros
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(mote_ros_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(mote_ros_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(mote_ros_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mote_ros)
  install(CODE "execute_process(COMMAND \"/home/wizard/miniconda3/envs/mote_ros/bin/python3.9\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mote_ros\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mote_ros
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(mote_ros_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(mote_ros_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(mote_ros_generate_messages_py sensor_msgs_generate_messages_py)
endif()
