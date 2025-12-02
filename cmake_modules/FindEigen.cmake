# FindEigen.cmake
# 包装脚本：让 PCL 1.11 能够找到系统安装的 Eigen3

# 首先尝试查找 Eigen3
find_package(Eigen3 QUIET)

if(EIGEN3_FOUND)
    # 如果找到 Eigen3，设置 Eigen 的兼容变量
    set(EIGEN_FOUND TRUE)
    set(EIGEN_INCLUDE_DIR ${EIGEN3_INCLUDE_DIR})
    set(EIGEN_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
    set(EIGEN_VERSION ${EIGEN3_VERSION})
    message(STATUS "Found Eigen via Eigen3: ${EIGEN3_INCLUDE_DIR}")
else()
    # 回退：手动查找
    find_path(EIGEN_INCLUDE_DIR
        NAMES Eigen/Core
        PATHS
            /usr/include/eigen3
            /usr/local/include/eigen3
            /opt/local/include/eigen3
    )
    
    if(EIGEN_INCLUDE_DIR)
        set(EIGEN_FOUND TRUE)
        set(EIGEN_INCLUDE_DIRS ${EIGEN_INCLUDE_DIR})
        message(STATUS "Found Eigen: ${EIGEN_INCLUDE_DIR}")
    else()
        set(EIGEN_FOUND FALSE)
        message(STATUS "Eigen not found")
    endif()
endif()

if(EIGEN_FOUND)
    # 设置 EIGEN_ROOT（PCL 需要的变量）
    # 优先使用环境变量 EIGEN_ROOT
    if(NOT DEFINED EIGEN_ROOT AND DEFINED ENV{EIGEN_ROOT})
        set(EIGEN_ROOT "$ENV{EIGEN_ROOT}")
        message(STATUS "Using EIGEN_ROOT from environment: ${EIGEN_ROOT}")
    elseif(NOT DEFINED EIGEN_ROOT)
        get_filename_component(EIGEN_ROOT "${EIGEN_INCLUDE_DIRS}" DIRECTORY)
        message(STATUS "Setting EIGEN_ROOT to: ${EIGEN_ROOT}")
    endif()
endif()

# Mark variables as advanced
mark_as_advanced(EIGEN_INCLUDE_DIR EIGEN_INCLUDE_DIRS EIGEN_VERSION EIGEN_ROOT)

