# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "traditional_stereo_ros: 1 messages, 0 services")

set(MSG_I_FLAGS "-Itraditional_stereo_ros:/home/anyone/catkin_ws_trevor/src/traditional_stereo_ros/msg;-Isensor_msgs:/opt/ros/melodic/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(traditional_stereo_ros_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/anyone/catkin_ws_trevor/src/traditional_stereo_ros/msg/Image.msg" NAME_WE)
add_custom_target(_traditional_stereo_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "traditional_stereo_ros" "/home/anyone/catkin_ws_trevor/src/traditional_stereo_ros/msg/Image.msg" "sensor_msgs/Image:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(traditional_stereo_ros
  "/home/anyone/catkin_ws_trevor/src/traditional_stereo_ros/msg/Image.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/traditional_stereo_ros
)

### Generating Services

### Generating Module File
_generate_module_cpp(traditional_stereo_ros
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/traditional_stereo_ros
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(traditional_stereo_ros_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(traditional_stereo_ros_generate_messages traditional_stereo_ros_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/anyone/catkin_ws_trevor/src/traditional_stereo_ros/msg/Image.msg" NAME_WE)
add_dependencies(traditional_stereo_ros_generate_messages_cpp _traditional_stereo_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(traditional_stereo_ros_gencpp)
add_dependencies(traditional_stereo_ros_gencpp traditional_stereo_ros_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS traditional_stereo_ros_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(traditional_stereo_ros
  "/home/anyone/catkin_ws_trevor/src/traditional_stereo_ros/msg/Image.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/traditional_stereo_ros
)

### Generating Services

### Generating Module File
_generate_module_eus(traditional_stereo_ros
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/traditional_stereo_ros
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(traditional_stereo_ros_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(traditional_stereo_ros_generate_messages traditional_stereo_ros_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/anyone/catkin_ws_trevor/src/traditional_stereo_ros/msg/Image.msg" NAME_WE)
add_dependencies(traditional_stereo_ros_generate_messages_eus _traditional_stereo_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(traditional_stereo_ros_geneus)
add_dependencies(traditional_stereo_ros_geneus traditional_stereo_ros_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS traditional_stereo_ros_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(traditional_stereo_ros
  "/home/anyone/catkin_ws_trevor/src/traditional_stereo_ros/msg/Image.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/traditional_stereo_ros
)

### Generating Services

### Generating Module File
_generate_module_lisp(traditional_stereo_ros
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/traditional_stereo_ros
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(traditional_stereo_ros_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(traditional_stereo_ros_generate_messages traditional_stereo_ros_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/anyone/catkin_ws_trevor/src/traditional_stereo_ros/msg/Image.msg" NAME_WE)
add_dependencies(traditional_stereo_ros_generate_messages_lisp _traditional_stereo_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(traditional_stereo_ros_genlisp)
add_dependencies(traditional_stereo_ros_genlisp traditional_stereo_ros_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS traditional_stereo_ros_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(traditional_stereo_ros
  "/home/anyone/catkin_ws_trevor/src/traditional_stereo_ros/msg/Image.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/traditional_stereo_ros
)

### Generating Services

### Generating Module File
_generate_module_nodejs(traditional_stereo_ros
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/traditional_stereo_ros
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(traditional_stereo_ros_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(traditional_stereo_ros_generate_messages traditional_stereo_ros_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/anyone/catkin_ws_trevor/src/traditional_stereo_ros/msg/Image.msg" NAME_WE)
add_dependencies(traditional_stereo_ros_generate_messages_nodejs _traditional_stereo_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(traditional_stereo_ros_gennodejs)
add_dependencies(traditional_stereo_ros_gennodejs traditional_stereo_ros_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS traditional_stereo_ros_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(traditional_stereo_ros
  "/home/anyone/catkin_ws_trevor/src/traditional_stereo_ros/msg/Image.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/traditional_stereo_ros
)

### Generating Services

### Generating Module File
_generate_module_py(traditional_stereo_ros
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/traditional_stereo_ros
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(traditional_stereo_ros_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(traditional_stereo_ros_generate_messages traditional_stereo_ros_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/anyone/catkin_ws_trevor/src/traditional_stereo_ros/msg/Image.msg" NAME_WE)
add_dependencies(traditional_stereo_ros_generate_messages_py _traditional_stereo_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(traditional_stereo_ros_genpy)
add_dependencies(traditional_stereo_ros_genpy traditional_stereo_ros_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS traditional_stereo_ros_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/traditional_stereo_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/traditional_stereo_ros
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(traditional_stereo_ros_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/traditional_stereo_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/traditional_stereo_ros
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(traditional_stereo_ros_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/traditional_stereo_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/traditional_stereo_ros
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(traditional_stereo_ros_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/traditional_stereo_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/traditional_stereo_ros
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(traditional_stereo_ros_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/traditional_stereo_ros)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/traditional_stereo_ros\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/traditional_stereo_ros
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(traditional_stereo_ros_generate_messages_py sensor_msgs_generate_messages_py)
endif()
