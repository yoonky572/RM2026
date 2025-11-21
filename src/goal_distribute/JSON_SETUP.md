# JSON库安装指导

本项目使用 `nlohmann/json` 库来解析JSON配置文件。以下是安装方法。

## 方法1: 使用系统包管理器安装（推荐）

### Ubuntu/Debian系统

```bash
sudo apt-get update
sudo apt-get install nlohmann-json3-dev
```

安装后，CMake会自动找到库，无需额外配置。

### 验证安装

```bash
pkg-config --modversion nlohmann_json
```

如果显示版本号（如 `3.11.2`），说明安装成功。

## 方法2: 使用CMake FetchContent自动下载（无需手动安装）

如果系统未安装 `nlohmann/json`，CMake会在编译时自动从GitHub下载。这需要：

- CMake 3.14 或更高版本
- 网络连接（编译时）

**优点**：无需手动安装，自动管理版本  
**缺点**：首次编译需要网络连接

## 方法3: 手动下载到项目中

如果上述方法都不适用，可以手动下载：

```bash
cd ~/Code/Sentry25NAVI/src/goal_distribute
mkdir -p third_party
cd third_party
git clone https://github.com/nlohmann/json.git
cd json
git checkout v3.11.2
```

然后修改 `CMakeLists.txt`，在 `find_package` 之前添加：

```cmake
# 如果使用手动下载的版本
if(NOT nlohmann_json_FOUND)
    set(JSON_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/json/include")
    include_directories(${JSON_INCLUDE_DIR})
endif()
```

## 验证安装

编译项目以验证JSON库是否正确配置：

```bash
cd ~/Code/Sentry25NAVI
catkin_make
```

如果编译成功，说明JSON库配置正确。

## 常见问题

### 问题1: 找不到 nlohmann/json

**错误信息**：
```
Could not find a package configuration file provided by "nlohmann_json"
```

**解决方法**：
- 使用方法1安装系统包，或
- 确保CMake版本 >= 3.14（使用方法2），或
- 使用方法3手动下载

### 问题2: CMake版本过低

**错误信息**：
```
Unknown CMake command "FetchContent_Declare"
```

**解决方法**：
升级CMake到3.14或更高版本：
```bash
# Ubuntu 18.04/20.04
sudo apt-get install cmake

# 或从源码安装最新版本
wget https://github.com/Kitware/CMake/releases/download/v3.27.0/cmake-3.27.0.tar.gz
tar -xzf cmake-3.27.0.tar.gz
cd cmake-3.27.0
./bootstrap && make && sudo make install
```

### 问题3: 编译时网络问题

如果使用方法2但编译时无法访问GitHub，可以：
1. 使用方法1安装系统包
2. 使用方法3手动下载
3. 配置代理或使用镜像

## 推荐方案

**推荐使用方法1（系统包管理器）**，因为：
- 最简单，一键安装
- 系统统一管理，易于维护
- 无需网络连接即可编译

如果系统包管理器没有该库，再考虑方法2或方法3。

