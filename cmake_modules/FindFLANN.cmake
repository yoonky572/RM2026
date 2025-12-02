# FindFLANN.cmake
# 包装脚本：让 PCL 1.11 能够找到系统安装的 FLANN

# First try pkg-config
find_package(PkgConfig QUIET)
if(PKG_CONFIG_FOUND)
    pkg_check_modules(FLANN_PKG QUIET flann)
endif()

# Try to find FLANN headers
find_path(FLANN_INCLUDE_DIR
    NAMES flann/flann.hpp
    PATHS
        /usr/include
        /usr/local/include
        ${FLANN_ROOT}/include
        ${FLANN_PKG_INCLUDE_DIRS}
    PATH_SUFFIXES
        flann
)

# Try to find FLANN library (C++ library first)
find_library(FLANN_LIBRARY
    NAMES flann_cpp flann
    PATHS
        /usr/lib
        /usr/lib/x86_64-linux-gnu
        /usr/local/lib
        ${FLANN_ROOT}/lib
        ${FLANN_PKG_LIBRARY_DIRS}
)

# Also try to find flann library (C library)
find_library(FLANN_C_LIBRARY
    NAMES flann
    PATHS
        /usr/lib
        /usr/lib/x86_64-linux-gnu
        /usr/local/lib
)

# Check if FLANN was found
if(FLANN_INCLUDE_DIR AND FLANN_LIBRARY)
    set(FLANN_FOUND TRUE)
    set(FLANN_INCLUDE_DIRS ${FLANN_INCLUDE_DIR})
    # Include both C++ and C libraries if available
    if(FLANN_C_LIBRARY)
        set(FLANN_LIBRARIES ${FLANN_LIBRARY} ${FLANN_C_LIBRARY})
    else()
        set(FLANN_LIBRARIES ${FLANN_LIBRARY})
    endif()
    
    # Extract version if possible
    if(FLANN_PKG_VERSION)
        set(FLANN_VERSION ${FLANN_PKG_VERSION})
    else()
        set(FLANN_VERSION "1.9.1")  # Default version
    endif()
    
    message(STATUS "Found FLANN: ${FLANN_LIBRARY}")
    message(STATUS "FLANN include dir: ${FLANN_INCLUDE_DIRS}")
else()
    set(FLANN_FOUND FALSE)
    message(STATUS "FLANN not found")
endif()

# Set FLANN_ROOT for PCL compatibility
if(FLANN_FOUND AND NOT FLANN_ROOT)
    get_filename_component(FLANN_ROOT "${FLANN_INCLUDE_DIRS}" DIRECTORY)
    message(STATUS "Setting FLANN_ROOT to: ${FLANN_ROOT}")
endif()

# Mark variables as advanced
mark_as_advanced(FLANN_INCLUDE_DIR FLANN_LIBRARY FLANN_ROOT)

