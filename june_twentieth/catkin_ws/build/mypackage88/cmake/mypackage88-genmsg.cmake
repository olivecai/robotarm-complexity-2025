# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "mypackage88: 1 messages, 1 services")

set(MSG_I_FLAGS "-Imypackage88:/home/user/myfolder88/catkin_ws/src/mypackage88/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(mypackage88_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/user/myfolder88/catkin_ws/src/mypackage88/msg/Position.msg" NAME_WE)
add_custom_target(_mypackage88_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mypackage88" "/home/user/myfolder88/catkin_ws/src/mypackage88/msg/Position.msg" ""
)

get_filename_component(_filename "/home/user/myfolder88/catkin_ws/src/mypackage88/srv/AddTwoInts.srv" NAME_WE)
add_custom_target(_mypackage88_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mypackage88" "/home/user/myfolder88/catkin_ws/src/mypackage88/srv/AddTwoInts.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(mypackage88
  "/home/user/myfolder88/catkin_ws/src/mypackage88/msg/Position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mypackage88
)

### Generating Services
_generate_srv_cpp(mypackage88
  "/home/user/myfolder88/catkin_ws/src/mypackage88/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mypackage88
)

### Generating Module File
_generate_module_cpp(mypackage88
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mypackage88
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(mypackage88_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(mypackage88_generate_messages mypackage88_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/user/myfolder88/catkin_ws/src/mypackage88/msg/Position.msg" NAME_WE)
add_dependencies(mypackage88_generate_messages_cpp _mypackage88_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/myfolder88/catkin_ws/src/mypackage88/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(mypackage88_generate_messages_cpp _mypackage88_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mypackage88_gencpp)
add_dependencies(mypackage88_gencpp mypackage88_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mypackage88_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(mypackage88
  "/home/user/myfolder88/catkin_ws/src/mypackage88/msg/Position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mypackage88
)

### Generating Services
_generate_srv_eus(mypackage88
  "/home/user/myfolder88/catkin_ws/src/mypackage88/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mypackage88
)

### Generating Module File
_generate_module_eus(mypackage88
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mypackage88
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(mypackage88_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(mypackage88_generate_messages mypackage88_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/user/myfolder88/catkin_ws/src/mypackage88/msg/Position.msg" NAME_WE)
add_dependencies(mypackage88_generate_messages_eus _mypackage88_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/myfolder88/catkin_ws/src/mypackage88/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(mypackage88_generate_messages_eus _mypackage88_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mypackage88_geneus)
add_dependencies(mypackage88_geneus mypackage88_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mypackage88_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(mypackage88
  "/home/user/myfolder88/catkin_ws/src/mypackage88/msg/Position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mypackage88
)

### Generating Services
_generate_srv_lisp(mypackage88
  "/home/user/myfolder88/catkin_ws/src/mypackage88/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mypackage88
)

### Generating Module File
_generate_module_lisp(mypackage88
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mypackage88
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(mypackage88_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(mypackage88_generate_messages mypackage88_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/user/myfolder88/catkin_ws/src/mypackage88/msg/Position.msg" NAME_WE)
add_dependencies(mypackage88_generate_messages_lisp _mypackage88_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/myfolder88/catkin_ws/src/mypackage88/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(mypackage88_generate_messages_lisp _mypackage88_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mypackage88_genlisp)
add_dependencies(mypackage88_genlisp mypackage88_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mypackage88_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(mypackage88
  "/home/user/myfolder88/catkin_ws/src/mypackage88/msg/Position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mypackage88
)

### Generating Services
_generate_srv_nodejs(mypackage88
  "/home/user/myfolder88/catkin_ws/src/mypackage88/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mypackage88
)

### Generating Module File
_generate_module_nodejs(mypackage88
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mypackage88
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(mypackage88_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(mypackage88_generate_messages mypackage88_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/user/myfolder88/catkin_ws/src/mypackage88/msg/Position.msg" NAME_WE)
add_dependencies(mypackage88_generate_messages_nodejs _mypackage88_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/myfolder88/catkin_ws/src/mypackage88/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(mypackage88_generate_messages_nodejs _mypackage88_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mypackage88_gennodejs)
add_dependencies(mypackage88_gennodejs mypackage88_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mypackage88_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(mypackage88
  "/home/user/myfolder88/catkin_ws/src/mypackage88/msg/Position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mypackage88
)

### Generating Services
_generate_srv_py(mypackage88
  "/home/user/myfolder88/catkin_ws/src/mypackage88/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mypackage88
)

### Generating Module File
_generate_module_py(mypackage88
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mypackage88
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(mypackage88_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(mypackage88_generate_messages mypackage88_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/user/myfolder88/catkin_ws/src/mypackage88/msg/Position.msg" NAME_WE)
add_dependencies(mypackage88_generate_messages_py _mypackage88_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/myfolder88/catkin_ws/src/mypackage88/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(mypackage88_generate_messages_py _mypackage88_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mypackage88_genpy)
add_dependencies(mypackage88_genpy mypackage88_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mypackage88_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mypackage88)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mypackage88
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(mypackage88_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mypackage88)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mypackage88
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(mypackage88_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mypackage88)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mypackage88
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(mypackage88_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mypackage88)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mypackage88
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(mypackage88_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mypackage88)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mypackage88\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mypackage88
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(mypackage88_generate_messages_py std_msgs_generate_messages_py)
endif()
