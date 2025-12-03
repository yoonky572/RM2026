cmake_minimum_required(VERSION 3.0.2)
project(tracking)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  serial
  geometry_msgs
  tf2_ros
  costmap_prohibition_layer
)

## System dependencies are found with CMake's conventions
# tracking 不需要 PCL 1.11，使用系统 PCL 1.10
# 清除可能的 PCL 1.11.1 路径，确保使用系统 PCL 1.10
unset(PCL_DIR CACHE)
unset(PCL_ROOT CACHE)
# 从 CMAKE_PREFIX_PATH 中移除 PCL 1.11 的安装路径
# PCL 1.11 通过 make install 安装在 /usr/local
list(REMOVE_ITEM CMAKE_PREFIX_PATH "/usr/local")
# 显式设置使用系统 PCL 1.10 的配置路径
list(INSERT CMAKE_PREFIX_PATH 0 "/usr")
set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl" CACHE PATH "System PCL 1.10 directory")
find_package(PCL 1.10 REQUIRED)


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
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
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
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
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
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tracking
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/tracking.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(tracking src/tracking.cpp)
add_executable(serial src/serial.cpp)
## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# 只使用系统 PCL 1.10 的库，明确指定系统库路径
# 从 catkin_LIBRARIES 中过滤掉 PCL 1.11.1 的库和所有 -lpcl_* 标志
set(FILTERED_CATKIN_LIBS "")
foreach(lib ${catkin_LIBRARIES})
  # 过滤掉 PCL 1.11 的库路径（/usr/local/lib）和所有 -lpcl_* 标志（我们使用明确的系统库路径）
  if(NOT "${lib}" MATCHES "/usr/local/lib.*pcl" AND NOT "${lib}" MATCHES "^-lpcl_")
    list(APPEND FILTERED_CATKIN_LIBS ${lib})
  endif()
endforeach()

# 明确指定只使用系统 PCL 1.10 的库（不依赖 PCL_LIBRARIES，因为可能包含 PCL 1.11.1 的路径）
# tracking 需要的 PCL 库：common, io, search, segmentation
set(SYSTEM_PCL_LIBS_EXPLICIT
  /usr/lib/x86_64-linux-gnu/libpcl_common.so
  /usr/lib/x86_64-linux-gnu/libpcl_io.so
  /usr/lib/x86_64-linux-gnu/libpcl_search.so
  /usr/lib/x86_64-linux-gnu/libpcl_segmentation.so
  /usr/lib/x86_64-linux-gnu/libpcl_kdtree.so
  /usr/lib/x86_64-linux-gnu/libpcl_octree.so
)

target_link_libraries(tracking
  ${FILTERED_CATKIN_LIBS}
  ${SYSTEM_PCL_LIBS_EXPLICIT}
)

# 彻底移除 rpath 中的 PCL 1.11.1 路径
set_target_properties(tracking PROPERTIES
  BUILD_WITH_INSTALL_RPATH FALSE
  INSTALL_RPATH_USE_LINK_PATH FALSE
  BUILD_RPATH ""
  INSTALL_RPATH ""
  LINK_FLAGS "-Wl,--disable-new-dtags -Wl,-rpath-link,/usr/lib/x86_64-linux-gnu"
)

# 在链接后使用 chrpath 移除 PCL 1.11.1 的 rpath（如果存在）
find_program(CHRPATH chrpath)
if(CHRPATH)
  add_custom_command(TARGET tracking POST_BUILD
    COMMAND ${CHRPATH} -d $<TARGET_FILE:tracking> || true
    COMMENT "Removing rpath from tracking binary"
  )
endif()
target_link_libraries(serial
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_tracking.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
