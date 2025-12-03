cmake_minimum_required(VERSION 2.8.3)
project(sentry_simulation)

set(CMAKE_BUILD_TYPE Release)
set(BUILD_STATIC_LIBS ON)
set(BUILD_SHARED_LIBS OFF)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  pcl_ros
  livox_ros_driver2
)

find_package(OpenCV REQUIRED)
# sentry_simulation 不需要 PCL 1.11，使用系统 PCL 1.10
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

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  pcl_ros
  livox_ros_driver2
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  "${PROJECT_SOURCE_DIR}/include"
  ~/RMUC_simulation/src/sentry_simulation/include
  /usr/local/include # Location when using 'make system_install'
  /usr/include       # More usual location (e.g. when installing using a package)
  ~/ws_RMNavLib/devel/include
)

## Specify additional locations for library files
# 注意：不要添加 /usr/local/lib，因为那里有 PCL 1.11.1 的库
# 只使用系统库路径
link_directories(
  /usr/lib/x86_64-linux-gnu  # 系统 PCL 1.10 库路径
)

## Declare executables
add_executable(vehicle_simulator ${PROJECT_SOURCE_DIR}/src/vehicle_simulator.cpp)
add_executable(key_control ${PROJECT_SOURCE_DIR}/src/key_control.cpp)
add_executable(livox2rosmsg ${PROJECT_SOURCE_DIR}/src/livox2rosmsg.cpp)
add_executable(path2marker ${PROJECT_SOURCE_DIR}/src/path2marker.cpp)

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
# vehicle_simulator 需要的 PCL 库：common, io, filters (VoxelGrid)
set(SYSTEM_PCL_LIBS_EXPLICIT
  /usr/lib/x86_64-linux-gnu/libpcl_common.so
  /usr/lib/x86_64-linux-gnu/libpcl_io.so
  /usr/lib/x86_64-linux-gnu/libpcl_filters.so
  /usr/lib/x86_64-linux-gnu/libpcl_kdtree.so
  /usr/lib/x86_64-linux-gnu/libpcl_octree.so
)

# 在链接标志中明确添加系统 PCL 库路径（确保在链接命令中）
string(REPLACE ";" " " SYSTEM_PCL_LIBS_FLAGS "${SYSTEM_PCL_LIBS_EXPLICIT}")

# 过滤 OpenCV_LIBRARIES 中的 -lpcl_* 标志
set(FILTERED_OPENCV_LIBS "")
foreach(lib ${OpenCV_LIBRARIES})
  if(NOT "${lib}" MATCHES "^-lpcl_")
    list(APPEND FILTERED_OPENCV_LIBS ${lib})
  endif()
endforeach()

# 先链接其他依赖（不包含系统 PCL 库）
target_link_libraries(vehicle_simulator ${FILTERED_CATKIN_LIBS} ${FILTERED_OPENCV_LIBS})

# 在链接命令的最后明确添加系统 PCL 库路径（确保在所有 -lpcl_* 标志之后）
# 直接链接系统 PCL 库，确保它们在链接命令的最后
target_link_libraries(vehicle_simulator ${SYSTEM_PCL_LIBS_EXPLICIT})

# 彻底移除 rpath 中的 PCL 1.11.1 路径
# 从 rpath 中移除 /usr/local/lib（那里有 PCL 1.11.1 的库）
# 使用 -Wl,--exclude-libs 和明确的库路径来确保链接正确的库
set_target_properties(vehicle_simulator PROPERTIES
  BUILD_WITH_INSTALL_RPATH FALSE
  INSTALL_RPATH_USE_LINK_PATH FALSE
  BUILD_RPATH ""
  INSTALL_RPATH ""
  LINK_FLAGS "-Wl,--disable-new-dtags -Wl,-rpath-link,/usr/lib/x86_64-linux-gnu"
)

# 在链接后使用 chrpath 移除 PCL 1.11.1 的 rpath（如果存在）
find_program(CHRPATH chrpath)
if(CHRPATH)
  add_custom_command(TARGET vehicle_simulator POST_BUILD
    COMMAND ${CHRPATH} -d $<TARGET_FILE:vehicle_simulator> || true
    COMMENT "Removing rpath from vehicle_simulator binary"
  )
endif()

target_link_libraries(key_control ${FILTERED_CATKIN_LIBS} ${OpenCV_LIBRARIES} ${SYSTEM_PCL_LIBS_EXPLICIT})
set_target_properties(key_control PROPERTIES
  BUILD_RPATH ""
  INSTALL_RPATH ""
  LINK_FLAGS "-Wl,--disable-new-dtags"
)

target_link_libraries(livox2rosmsg ${FILTERED_CATKIN_LIBS} ${OpenCV_LIBRARIES} ${SYSTEM_PCL_LIBS_EXPLICIT})
set_target_properties(livox2rosmsg PROPERTIES
  BUILD_RPATH ""
  INSTALL_RPATH ""
  LINK_FLAGS "-Wl,--disable-new-dtags"
)
# 确保 livox_ros_driver2 的消息在编译前生成
add_dependencies(livox2rosmsg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} livox_ros_driver2_generate_messages_cpp)

target_link_libraries(path2marker ${FILTERED_CATKIN_LIBS} ${OpenCV_LIBRARIES} ${SYSTEM_PCL_LIBS_EXPLICIT})
set_target_properties(path2marker PROPERTIES
  BUILD_RPATH ""
  INSTALL_RPATH ""
  LINK_FLAGS "-Wl,--disable-new-dtags"
)

install(TARGETS vehicle_simulator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(DIRECTORY log/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/log
)
install(DIRECTORY mesh/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/mesh
)
install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)
install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)
install(DIRECTORY world/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/world
)

set(PYTHON_EXECUTABLE /usr/bin/python3)
catkin_install_python(PROGRAMS
  src/auto_obstacle.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_install_python(PROGRAMS
  src/launch_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
# catkin_install_python(PROGRAMS
#   src/backup.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

