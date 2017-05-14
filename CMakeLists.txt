cmake_minimum_required(VERSION 2.8.3)
project(itomp_nlp)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")

## show all files (http://wiki.ros.org/IDEs#QtCreator)
#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
FILE(GLOB children ${CMAKE_SOURCE_DIR})
FOREACH(child ${children})
  IF(IS_DIRECTORY ${child})
    file(GLOB_RECURSE dir_files "${child}/*")
    LIST(APPEND extra_files ${dir_files})
  ENDIF()
ENDFOREACH()
add_custom_target(dummy_${PROJECT_NAME} SOURCES ${extra_files})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# cmake_modules
find_package(cmake_modules REQUIRED)

# eigen
find_package(Eigen REQUIRED)

# assimp
find_package(assimp REQUIRED VERSION 3.3.1)

# Qt5
# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed
set(CMAKE_AUTOMOC ON)
# Find the Qt library
set(Qt5Widgets_DIR /home/jaesungp/Qt/5.8/gcc_64)
set(Qt5OpenGL_DIR /home/jaesungp/Qt/5.8/gcc_64)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5OpenGL REQUIRED)
add_definitions(${Qt5Widgets_DEFINITIONS})
add_definitions(${Qt5OpenGL_DEFINITIONS})
include_directories(${Qt5Widgets_INCLUDE_DIRS})
include_directories(${Qt5OpenGL_INCLUDE_DIRS})

message(${CMAKE_PREFIX_PATH})
message(${Qt5Widgets_INCLUDE_DIRS})
message(${Qt5OpenGL_INCLUDE_DIRS})


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
  #INCLUDE_DIRS include
  #LIBRARIES itomp_nlp
  #CATKIN_DEPENDS roscpp actionlib
  #DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  lib
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${ASSIMP_INCLUDE_DIRS}
  ${ZMQ_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5OpenGL_INCLUDE_DIRS}
)

link_directories(
  /usr/local/lib
  lib
)

# Qt5 moc
qt5_wrap_cpp(interface_moc
  include/itomp_nlp/interface/itomp_cost_function_widget.h
  include/itomp_nlp/interface/itomp_cost_functions_widget.h
  include/itomp_nlp/interface/itomp_datagen_widget.h
  include/itomp_nlp/interface/itomp_interface.h
  include/itomp_nlp/interface/itomp_nlp_widget.h
  include/itomp_nlp/interface/main_window.h
  include/itomp_nlp/renderer/renderer.h
)

message(${interface_moc})

# collect .h files
file(GLOB_RECURSE itomp_HEADER_FILES
  include/itomp_nlp/*.h
  lib/*.h
)
file(GLOB_RECURSE itomp_EXCLUDING_HEADER_FILES
  # Qt interface
  include/itomp_nlp/interface/itomp_cost_function_widget.h
  include/itomp_nlp/interface/itomp_cost_functions_widget.h
  include/itomp_nlp/interface/itomp_datagen_widget.h
  include/itomp_nlp/interface/itomp_interface.h
  include/itomp_nlp/interface/itomp_nlp_widget.h
  include/itomp_nlp/interface/main_window.h
  include/itomp_nlp/renderer/renderer.h
)
list(REMOVE_ITEM itomp_HEADER_FILES
  ${itomp_EXCLUDING_HEADER_FILES}
)

## Declare a C++ library
add_library(itomp_nlp
  # renderer
  src/renderer/camera.cpp
  src/renderer/color_shader.cpp
  src/renderer/gl_base.cpp
  src/renderer/light.cpp
  src/renderer/light_oit_shader.cpp
  src/renderer/light_shader.cpp
  src/renderer/light_shadow_shader.cpp
  src/renderer/material.cpp
  src/renderer/normal_shader.cpp
  src/renderer/oit_resolve_shader.cpp
  src/renderer/renderer.cpp
  src/renderer/rendering_box.cpp
  src/renderer/rendering_capsule.cpp
  src/renderer/rendering_human.cpp
  #src/renderer/rendering_kinect_human.cpp
  #src/renderer/rendering_kinect_points.cpp
  src/renderer/rendering_mesh.cpp
  src/renderer/rendering_mesh_manager.cpp
  src/renderer/rendering_plane.cpp
  src/renderer/rendering_robot.cpp
  src/renderer/rendering_robot_trajectory.cpp
  src/renderer/rendering_shape.cpp
  src/renderer/shader_program.cpp
  src/renderer/shadowmap_point_shader.cpp
  src/renderer/shadowmap_shader.cpp
  src/renderer/texture.cpp
  src/renderer/wireframe_shader.cpp
  ${interface_moc}

  # shape
  src/shape/aabb.cpp
  src/shape/capsule.cpp
  src/shape/capsule2.cpp
  src/shape/mesh.cpp
  src/shape/mesh_loader.cpp
  src/shape/obb.cpp
  src/shape/rigid_body.cpp
  src/shape/shape.cpp

  # robot
  src/robot/continuous_joint.cpp
  src/robot/fixed_joint.cpp
  src/robot/joint.cpp
  src/robot/link.cpp
  src/robot/prismatic_joint.cpp
  src/robot/revolute_joint.cpp
  src/robot/robot_model.cpp
  src/robot/robot_state.cpp
  src/robot/urdf_parser.cpp

  # optimization
  src/optimization/optimizer.cpp
  src/optimization/optimizer_robot.cpp
  src/optimization/optimizer_robot_loader.cpp
  src/optimization/optimizer_thread.cpp
  src/optimization/cost.cpp
  src/optimization/collision_cost.cpp
  src/optimization/goal_cost.cpp
  src/optimization/goal_orientation_cost.cpp
  src/optimization/goal_region_cost.cpp
  src/optimization/goal_upvector_cost.cpp
  src/optimization/repulsive_cost.cpp
  src/optimization/smoothness_cost.cpp
  src/optimization/velocity_cost.cpp
  src/optimization/scene.cpp
  src/optimization/static_obstacle.cpp
  src/optimization/dynamic_obstacle.cpp
  #src/optimization/dynamic_kf_human_obstacle.cpp
  src/optimization/trajectory.cpp
  src/optimization/trajectory_publisher_thread.cpp

  # network
  src/network/trajectory_publisher.cpp

  # interface
  src/interface/itomp_cost_function_widget.cpp
  src/interface/itomp_cost_functions_widget.cpp
  src/interface/itomp_datagen_widget.cpp
  src/interface/itomp_interface.cpp
  src/interface/itomp_nlp_widget.cpp
  src/interface/main_window.cpp

  # utils
  src/utils/conversion.cpp
  src/utils/timing.cpp

  # lodepng
  lib/lodepng.cpp

  # tinyxml2
  lib/tinyxml2.cpp

  # header files for QtCreator import
  ${itomp_HEADER_FILES}
)

target_link_libraries(itomp_nlp
  ${Qt5Widgets_LIBRARIES}
  ${Qt5OpenGL_LIBRARIES}
  ${ASSIMP_LIBRARIES}
  ${ZMQ_LIBRARIES}
  zmq
  speech_subscriber
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(itomp_nlp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(test_interface
  src/test/test_interface.cpp
)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(itomp_nlp_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(test_interface
  itomp_nlp
  ${catkin_LIBRARIES}
)

# copy shaders
file(COPY shader/ DESTINATION ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}/shader)

## Declare a C++ executable
add_executable(itomp_traj_execute
  include/itomp_nlp/network/trajectory_subscriber.h
  include/itomp_nlp/optimization/trajectory.h
  include/itomp_nlp/ros/fetch.h

  src/network/trajectory_subscriber.cpp
  src/optimization/trajectory.cpp
  src/ros/fetch.cpp
  src/ros/trajectory_executor.cpp
)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(itomp_nlp_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(itomp_traj_execute
  zmq
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS itomp_nlp itomp_nlp_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_itomp_nlp.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
