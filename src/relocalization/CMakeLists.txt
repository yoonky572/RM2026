cmake_minimum_required(VERSION 3.0.2)
project(relocalization)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
# ========== relocalization 使用 PCL 1.11 ==========
# small_gicp 需要 std::shared_ptr，PCL 1.11 使用 std::shared_ptr，而 PCL 1.10 使用 boost::shared_ptr
# 清除之前可能的缓存，确保使用 PCL 1.11
unset(PCL_DIR CACHE)
unset(PCL_ROOT CACHE)

# 设置PCL 1.11的cmake配置文件路径
# PCL 1.11通过make install后，配置文件在/usr/local/share/pcl-1.11
set(PCL_DIR "/usr/local/share/pcl-1.11" CACHE PATH "PCL 1.11 directory" FORCE)

# 验证PCL配置文件是否存在
if(NOT EXISTS "${PCL_DIR}/PCLConfig.cmake")
  message(FATAL_ERROR "PCL 1.11配置文件未找到: ${PCL_DIR}/PCLConfig.cmake")
endif()

# 先查找 Eigen3（PCL 1.11 需要，必须在 PCL 之前查找）
find_package(Eigen3 REQUIRED)
message(STATUS "Eigen3 found: ${EIGEN3_INCLUDE_DIR}")

# 使用PCL 1.11版本，必须使用1.11或更高版本以支持std::shared_ptr
# 注意：filters 组件是必需的，因为代码中使用了 StatisticalOutlierRemoval
find_package(PCL 1.11 REQUIRED COMPONENTS common kdtree io filters features search sample_consensus octree)

# 验证PCL版本
if(PCL_VERSION VERSION_LESS "1.11")
  message(FATAL_ERROR "relocalization包需要PCL 1.11或更高版本（small_gicp需要std::shared_ptr），但找到的是 ${PCL_VERSION}")
endif()

# 输出 PCL 版本信息用于验证
message(STATUS "=== relocalization包PCL配置 ===")
message(STATUS "PCL_VERSION = ${PCL_VERSION}")
message(STATUS "PCL_LIBRARY_DIRS = ${PCL_LIBRARY_DIRS}")
message(STATUS "PCL_INCLUDE_DIRS = ${PCL_INCLUDE_DIRS}")
message(STATUS "PCL_DIR = ${PCL_DIR}")
# file(GLOB PCL_LIBRARIES /home/tjurm/MylibInstall/pcl-1.11.0/lib/libpcl_* usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
# /usr/lib/x86_64-linux-gnu/libboost_system.so /usr/lib/x86_64-linux-gnu/libboost_filesystem.so /usr/lib/x86_64-linux-gnu/libboost_date_time.so
# /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
# /usr/lib/x86_64-linux-gnu/libboost_serialization.so /usr/lib/x86_64-linux-gnu/libboost_regex.so)
# set(PCL_INCLUDE_DIRS /home/tjurm/MylibInstall/pcl-1.11.0/include/pcl-1.11 /usr/include/eigen3 
# /usr/include/ni /usr/include/openni2 /usr/include/vtk-7.1 /usr/include/freetype2 /usr/include/x86_64-linux-gnu )


message(STATUS "PCL_INCLUDE_DIRS = ${PCL_INCLUDE_DIRS}")
find_package(OpenMP REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf2_ros
)
message(STATUS "PCL_VERSION = ${PCL_VERSION}")
message(STATUS "PCL_LIBRARIES = ${PCL_LIBRARIES}")
message(STATUS "CATKIN_LIBRARIES = ${catkin_LIBRARIES}")

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
#  LIBRARIES relocalization
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)
## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/relocalization.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(relocalization src/relocalization.cpp)

# 定义宏以支持 PCL 1.10（small_gicp 兼容性）
target_compile_definitions(relocalization PRIVATE SMALL_GICP_ALLOW_PCL_1_10)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# 确保正确链接 PCL 1.11 的库（从 /usr/local/lib）
# 从 catkin_LIBRARIES 中过滤掉可能存在的 PCL 1.10 库路径
set(FILTERED_CATKIN_LIBS "")
foreach(lib ${catkin_LIBRARIES})
  # 过滤掉系统 PCL 1.10 的库路径，确保只使用 PCL 1.11
  if(NOT "${lib}" MATCHES "/usr/lib/x86_64-linux-gnu/libpcl_" AND NOT "${lib}" MATCHES "^-lpcl_")
    list(APPEND FILTERED_CATKIN_LIBS ${lib})
  endif()
endforeach()

# 显式指定 PCL 1.11 的库（使用完整路径，避免链接器找到系统 PCL 1.10 的库）
# relocalization 需要的 PCL 库：common, kdtree, io, filters, features, search, sample_consensus, octree
set(PCL_1_11_LIBS_EXPLICIT
  ${PCL_LIBRARY_DIRS}/libpcl_common.so
  ${PCL_LIBRARY_DIRS}/libpcl_kdtree.so
  ${PCL_LIBRARY_DIRS}/libpcl_io.so
  ${PCL_LIBRARY_DIRS}/libpcl_filters.so
  ${PCL_LIBRARY_DIRS}/libpcl_features.so
  ${PCL_LIBRARY_DIRS}/libpcl_search.so
  ${PCL_LIBRARY_DIRS}/libpcl_sample_consensus.so
  ${PCL_LIBRARY_DIRS}/libpcl_octree.so
)

# 从 PCL_LIBRARIES 中提取非 PCL 库（如 boost, vtk, FLANN 等）
set(PCL_OTHER_LIBS "")
foreach(lib ${PCL_LIBRARIES})
  # 保留非 pcl_ 开头的库（boost, vtk, FLANN 等）
  if(NOT "${lib}" MATCHES "^pcl_")
    list(APPEND PCL_OTHER_LIBS ${lib})
  endif()
endforeach()

# 链接库，确保使用 PCL 1.11 的库
target_link_libraries(${PROJECT_NAME}
  PUBLIC 
  ${FILTERED_CATKIN_LIBS}
  ${PCL_1_11_LIBS_EXPLICIT}
  ${PCL_OTHER_LIBS}
  OpenMP::OpenMP_CXX
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_relocalization.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
