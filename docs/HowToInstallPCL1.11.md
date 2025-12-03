# PCL 1.11 安装指南

本文档介绍如何在 Ubuntu 系统上从源码编译和安装 PCL (Point Cloud Library) 1.11。

## 目录

- [系统要求](#系统要求)
- [安装依赖](#安装依赖)
- [下载源码](#下载源码)
- [编译和安装](#编译和安装)
- [验证安装](#验证安装)
- [配置 CMake 项目](#配置-cmake-项目)
- [常见问题](#常见问题)

## 系统要求

- **操作系统**: Ubuntu 18.04 / 20.04 / 22.04
- **编译器**: GCC 7+ 或 Clang 5+
- **CMake**: 3.5 或更高版本
- **内存**: 建议至少 4GB RAM（编译时）
- **磁盘空间**: 至少 2GB 可用空间

## 安装依赖

### 1. 更新系统包

```bash
sudo apt update
sudo apt upgrade -y
```

### 2. 安装基础构建工具

```bash
sudo apt install -y build-essential cmake git
```

### 3. 安装 PCL 依赖库

```bash
# 基础依赖
sudo apt install -y libboost-all-dev libeigen3-dev libflann-dev

# VTK 依赖（用于可视化，可选但推荐）
sudo apt install -y libvtk7-dev

# OpenNI 和 OpenNI2（用于深度相机支持）
sudo apt install -y libopenni-dev libopenni2-dev

# 其他依赖
sudo apt install -y libqhull-dev libpcap-dev libusb-1.0-0-dev

# 可选：CUDA 支持（如果有 NVIDIA GPU）
# sudo apt install -y nvidia-cuda-toolkit
```

### 4. 安装可选依赖（推荐）

```bash
# PNG, JPEG, TIFF 支持
sudo apt install -y libpng-dev libjpeg-dev libtiff-dev

# OpenGL 支持
sudo apt install -y libgl1-mesa-dev libglu1-mesa-dev

# 其他工具
sudo apt install -y libproj-dev libdoxygen-dev
```

## 下载源码

### 方法 1: 从 GitHub 下载（推荐）

```bash
# 创建工作目录
mkdir -p ~/pcl_build
cd ~/pcl_build

# 克隆 PCL 源码
git clone https://github.com/PointCloudLibrary/pcl.git
cd pcl

# 切换到 PCL 1.11.1 标签（tag）
git checkout pcl-1.11.1
```

**注意**: `pcl-1.11.1` 是一个 Git 标签（tag），不是分支。如果需要其他 1.11.x 版本，可以查看所有可用标签：

```bash
# 查看所有 1.11.x 版本的标签
git tag | grep "pcl-1.11"

# 切换到其他版本，例如：
# git checkout pcl-1.11.0
```

### 方法 2: 下载压缩包

```bash
# 创建工作目录
mkdir -p ~/pcl_build
cd ~/pcl_build

# 下载源码压缩包
wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.11.1.tar.gz
tar -xzf pcl-1.11.1.tar.gz
cd pcl-pcl-1.11.1
```

## 编译和安装

### 1. 创建构建目录

```bash
cd ~/pcl_build/pcl  # 或 pcl-pcl-1.11.1
mkdir build
cd build
```

### 2. 配置 CMake

```bash
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DBUILD_SHARED_LIBS=ON \
  -DWITH_CUDA=OFF \
  -DWITH_OPENGL=ON \
  -DWITH_QT=OFF \
  -DWITH_VTK=ON \
  -DWITH_PCAP=ON \
  -DWITH_OPENNI=ON \
  -DWITH_OPENNI2=ON \
  -DWITH_LIBUSB=ON
```

**配置选项说明：**

- `CMAKE_BUILD_TYPE=Release`: 使用 Release 模式编译（性能更好）
- `CMAKE_INSTALL_PREFIX=/usr/local`: 安装到 `/usr/local`（系统默认位置）
- `BUILD_SHARED_LIBS=ON`: 构建共享库（推荐）
- `WITH_CUDA=OFF`: 禁用 CUDA（如果有 GPU 可设为 ON）
- `WITH_OPENGL=ON`: 启用 OpenGL 支持
- `WITH_QT=OFF`: 禁用 Qt（如果需要可视化工具可设为 ON）
- `WITH_VTK=ON`: 启用 VTK 支持（用于可视化）
- `WITH_PCAP=ON`: 启用 PCAP 支持（用于读取 PCD 文件）
- `WITH_OPENNI=ON`: 启用 OpenNI 支持
- `WITH_OPENNI2=ON`: 启用 OpenNI2 支持
- `WITH_LIBUSB=ON`: 启用 USB 设备支持

### 3. 编译

```bash
# 使用所有可用 CPU 核心编译（推荐）
make -j$(nproc)

# 或者指定线程数（例如使用 4 个线程）
# make -j4
```

**注意：** 编译过程可能需要 30-60 分钟，取决于系统性能。

### 4. 安装

```bash
sudo make install
```

安装完成后，PCL 1.11 将被安装到 `/usr/local` 目录：
- 头文件: `/usr/local/include/pcl-1.11`
- 库文件: `/usr/local/lib`
- CMake 配置: `/usr/local/share/pcl-1.11`

### 5. 更新动态链接库缓存

```bash
sudo ldconfig
```

## 验证安装

### 1. 检查版本

```bash
pcl_version --version
```

如果命令不存在，可以检查库文件：

```bash
ls -la /usr/local/lib/libpcl_common.so*
```

应该看到类似输出：
```
libpcl_common.so -> libpcl_common.so.1.11
libpcl_common.so.1.11 -> libpcl_common.so.1.11.1
libpcl_common.so.1.11.1
```

### 2. 检查 CMake 配置

```bash
ls -la /usr/local/share/pcl-1.11/
```

应该看到 `PCLConfig.cmake` 文件。

### 3. 测试编译

创建测试文件 `test_pcl.cpp`:

```cpp
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>

int main() {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    std::cout << "PCL version: " << PCL_VERSION_PRETTY << std::endl;
    std::cout << "PCL installed successfully!" << std::endl;
    return 0;
}
```

编译测试：

```bash
g++ -std=c++14 test_pcl.cpp -o test_pcl \
  -I/usr/local/include/pcl-1.11 \
  -I/usr/include/eigen3 \
  -L/usr/local/lib \
  -lpcl_common

./test_pcl
```

应该输出 PCL 版本信息。

## 配置 CMake 项目

### 在 CMakeLists.txt 中使用 PCL 1.11

```cmake
# 清除之前可能的 PCL 缓存
unset(PCL_DIR CACHE)
unset(PCL_ROOT CACHE)

# 设置 PCL 1.11 的 cmake 配置文件路径
set(PCL_DIR "/usr/local/share/pcl-1.11" CACHE PATH "PCL 1.11 directory" FORCE)

# 验证 PCL 配置文件是否存在
if(NOT EXISTS "${PCL_DIR}/PCLConfig.cmake")
  message(FATAL_ERROR "PCL 1.11配置文件未找到: ${PCL_DIR}/PCLConfig.cmake")
endif()

# 先查找 Eigen3（PCL 1.11 需要，必须在 PCL 之前查找）
find_package(Eigen3 REQUIRED)

# 使用 PCL 1.11 版本
find_package(PCL 1.11 REQUIRED COMPONENTS common kdtree io filters)

# 验证 PCL 版本
if(PCL_VERSION VERSION_LESS "1.11")
  message(FATAL_ERROR "需要PCL 1.11或更高版本，但找到的是 ${PCL_VERSION}")
endif()

# 包含头文件
include_directories(${PCL_INCLUDE_DIRS})

# 链接库
target_link_libraries(your_target ${PCL_LIBRARIES})
```

### 显式链接 PCL 1.11 库（避免链接到系统 PCL 1.10）

如果系统同时安装了 PCL 1.10 和 PCL 1.11，需要显式指定库路径：

```cmake
# 显式指定 PCL 1.11 的库（使用完整路径）
set(PCL_1_11_LIBS_EXPLICIT
  ${PCL_LIBRARY_DIRS}/libpcl_common.so
  ${PCL_LIBRARY_DIRS}/libpcl_kdtree.so
  ${PCL_LIBRARY_DIRS}/libpcl_io.so
  ${PCL_LIBRARY_DIRS}/libpcl_filters.so
)

# 从 PCL_LIBRARIES 中提取非 PCL 库（如 boost, vtk 等）
set(PCL_OTHER_LIBS "")
foreach(lib ${PCL_LIBRARIES})
  if(NOT "${lib}" MATCHES "^pcl_")
    list(APPEND PCL_OTHER_LIBS ${lib})
  endif()
endforeach()

# 链接库
target_link_libraries(your_target
  ${PCL_1_11_LIBS_EXPLICIT}
  ${PCL_OTHER_LIBS}
)
```

## 常见问题

### 1. 编译时找不到 Eigen3

**错误信息：**
```
Could not find a package configuration file provided by "Eigen"
```

**解决方法：**
```bash
sudo apt install -y libeigen3-dev
```

在 CMakeLists.txt 中，确保在查找 PCL 之前先查找 Eigen3：
```cmake
find_package(Eigen3 REQUIRED)
find_package(PCL 1.11 REQUIRED ...)
```

### 2. 链接时找不到 PCL 库

**错误信息：**
```
undefined reference to `pcl::...`
```

**解决方法：**
- 确保 `PCL_LIBRARIES` 包含了所需的库
- 检查是否链接了正确的 PCL 1.11 库（而不是系统 PCL 1.10）
- 使用 `ldconfig` 更新库缓存

### 3. 运行时找不到库

**错误信息：**
```
error while loading shared libraries: libpcl_common.so.1.11: cannot open shared object file
```

**解决方法：**
```bash
# 更新动态链接库缓存
sudo ldconfig

# 或者设置 LD_LIBRARY_PATH（临时）
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```

### 4. CMake 找到错误的 PCL 版本

**解决方法：**
在 CMakeLists.txt 中显式设置 PCL_DIR：
```cmake
unset(PCL_DIR CACHE)
set(PCL_DIR "/usr/local/share/pcl-1.11" CACHE PATH "PCL 1.11 directory" FORCE)
```

### 5. 编译时间过长

**解决方法：**
- 使用多线程编译：`make -j$(nproc)`
- 禁用不需要的模块（如 CUDA、Qt 等）
- 使用 Release 模式而不是 Debug 模式

### 6. 内存不足

**解决方法：**
- 减少编译线程数：`make -j2` 或 `make -j4`
- 关闭其他占用内存的程序
- 增加系统交换空间

## 卸载

如果需要卸载 PCL 1.11：

```bash
# 进入构建目录
cd ~/pcl_build/pcl/build

# 卸载（需要 root 权限）
sudo make uninstall

# 手动删除残留文件（如果 make uninstall 不完整）
sudo rm -rf /usr/local/include/pcl-1.11
sudo rm -rf /usr/local/lib/libpcl_*
sudo rm -rf /usr/local/share/pcl-1.11
sudo rm -rf /usr/local/lib/cmake/pcl-1.11

# 更新库缓存
sudo ldconfig
```

## 参考资源

- [PCL 官方文档](https://pcl.readthedocs.io/)
- [PCL GitHub 仓库](https://github.com/PointCloudLibrary/pcl)
- [PCL 编译指南](https://pcl.readthedocs.io/projects/tutorials/en/latest/compiling_pcl_posix.html)

## 版本信息

- **文档版本**: 1.0
- **PCL 版本**: 1.11.1
- **最后更新**: 2024年

---

**注意**: 如果遇到其他问题，请检查 PCL 官方文档或提交 Issue 到 PCL GitHub 仓库。

