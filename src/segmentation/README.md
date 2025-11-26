# Segmentation 障碍物检测包

## 概述

本包实现了一个基于点云配准的动态障碍物检测系统，通过比较当前扫描点云与先验地图，检测出动态障碍物（不在先验地图中的点）。

## 功能特性

- **动态障碍物检测**: 使用 KD 树查找最近邻，检测距离地图较远的点作为动态障碍物
- **点云配准**: 可选的点云配准功能，提高检测精度
- **实时处理**: 支持实时处理激光雷达扫描和障碍物点云
- **自适应裁剪**: 根据机器人位置自动裁剪处理区域，提高效率
- **匹配状态检测**: 检测点云匹配状态，判断定位是否发散

## 代码结构

```
segmentation/
├── CMakeLists.txt              # 构建配置
├── package.xml                  # 包配置
├── README.md                   # 本文档
├── config/
│   └── extrinsic.yaml         # 外参配置文件
├── launch/
│   └── obstacle_detect.launch  # 启动文件
├── include/
│   └── segmentation/
│       ├── segmentation.hpp    # 类头文件
│       └── utility.hpp         # 工具函数
└── src/
    └── segmentation.cpp        # 类实现
```

## 依赖项

### ROS 包依赖
- `roscpp`: ROS C++ 客户端库
- `tf`: TF 库
- `sensor_msgs`: 传感器消息类型
- `geometry_msgs`: 几何消息类型
- `std_msgs`: 标准消息类型
- `nav_msgs`: 导航消息类型

### 第三方库依赖
- **PCL 1.11**: 点云处理库
  - `pcl_common`
  - `pcl_kdtree`
  - `pcl_io`
  - `pcl_filters`
- **Eigen3**: 线性代数库
- **OpenMP**: 并行计算库
- **small_gicp**: 快速 GICP 实现（包含在包中）

## 编译

```bash
cd ~/catkin_ws
catkin_make
# 或
catkin build segmentation
```

**注意**: 本包需要特定版本的 PCL 1.11，CMakeLists.txt 中已配置了自定义的 PCL 路径。请根据您的系统环境调整路径。

## 配置

### 启动文件参数

在 `launch/obstacle_detect.launch` 中配置以下参数：

#### 必需参数
- `map_path` (string): 先验地图 PCD 文件路径

#### 检测参数
- `distance_threshold` (double, 默认: 0.2): 障碍物检测距离阈值（米），超过此距离的点被认为是障碍物
- `leaf_size` (double, 默认: 0.05): 体素滤波的叶子大小（米）
- `box_size` (float, 默认: 1.5): 裁剪区域大小（米）

#### 匹配参数
- `diverge_threshold` (double, 默认: 0.5): 发散阈值（障碍物点百分比），超过此值认为匹配失败
- `high_match_threshold` (double, 默认: 0.2): 高匹配阈值（障碍物点百分比），低于此值认为匹配度高

#### 配准参数（可选）
- `align_en` (bool, 默认: false): 是否启用点云配准
- `max_dist_sq` (double, 默认: 1.0): 配准最大距离平方
- `max_iterations` (double, 默认: 100): 配准最大迭代次数

#### 激光雷达参数
- `lidar_height` (double, 默认: 0.145): 激光雷达高度（米）
- `lidar_roll` (double, 默认: -0.349): 激光雷达俯仰角（弧度）
- `lidar_y` (double, 默认: 0.11): 激光雷达 Y 偏移（米）

#### 其他参数
- `freq` (int, 默认: 10): 运行频率（Hz）
- `prior_map_pub_en` (bool, 默认: false): 是否发布先验地图
- `debug_en` (bool, 默认: false): 是否启用调试模式
- `use_livox_cloud` (bool, 默认: false): 是否使用 Livox 点云（未使用）

### 示例配置

```xml
<node pkg="segmentation" type="segmentation" name="detector" output="screen">
    <param name="map_path" value="/path/to/map.pcd"/>
    <param name="freq" value="20"/>
    <param name="distance_threshold" value="0.3"/>
    <param name="leaf_size" value="0.1"/>
    <param name="box_size" value="5.0"/>
    <param name="diverge_threshold" value="2.0"/>
    <param name="debug_en" value="true"/>
    <param name="align_en" value="false"/>
</node>
```

## 话题

### 订阅

- `/cloud_registered_body` (sensor_msgs/PointCloud2): 已配准的扫描点云（body 坐标系）
- `/ground_segmentation/obstacle_cloud` (sensor_msgs/PointCloud2): 障碍物点云

### 发布

- `obstacle` (sensor_msgs/PointCloud2): 检测到的动态障碍物点云（body 坐标系）
- `/diverge` (std_msgs/Bool): 发散标志（定位是否发散）
- `/match` (std_msgs/Bool): 匹配标志（点云是否匹配成功）
- `/body_pose` (geometry_msgs/PoseStamped): 机器人位姿（仅在匹配度高时发布）
- `prior_map` (sensor_msgs/PointCloud2): 先验地图点云（可选，仅在 `prior_map_pub_en=true` 时发布）
- `aligned` (sensor_msgs/PointCloud2): 配准后的点云（可选，仅在 `align_en=true` 时发布）

## 算法说明

### 障碍物检测流程

1. **点云预处理**:
   - 从传感器坐标系转换到地图坐标系
   - 体素滤波降采样
   - 裁剪到机器人周围区域（提高效率）

2. **可选配准**:
   - 如果启用配准（`align_en=true`），使用 GICP 进行点云配准
   - 提高点云对齐精度，减少误检

3. **障碍物检测**:
   - 对每个扫描点，在先验地图 KD 树中查找最近邻
   - 如果距离超过阈值（`distance_threshold`），认为是动态障碍物

4. **匹配状态判断**:
   - 计算障碍物点百分比
   - 如果百分比超过 `diverge_threshold`，认为匹配失败
   - 如果百分比低于 `high_match_threshold`，认为匹配度高
   - 如果匹配失败持续超过 3 秒，设置发散标志

5. **结果发布**:
   - 将障碍物点云转换回 body 坐标系并发布
   - 发布匹配和发散标志
   - 如果匹配度高，发布机器人位姿供重定位使用

### 自适应裁剪

系统会根据机器人位置自动调整处理区域：
- 初始裁剪区域中心在原点
- 当机器人移动超过 `box_size - 1` 米时，更新裁剪区域中心
- 只处理裁剪区域内的点云，提高处理效率

### 匹配状态说明

- **match**: 点云是否匹配成功（障碍物点百分比是否低于 `diverge_threshold`）
- **diverge**: 定位是否发散（匹配失败持续超过 3 秒）
- **high_match**: 匹配度是否高（障碍物点百分比是否低于 `high_match_threshold`）

## 使用示例

### 基本使用

1. 准备地图文件：
   - 生成先验地图 PCD 文件

2. 启动节点：
```bash
roslaunch segmentation obstacle_detect.launch
```

3. 在 RViz 中可视化：
   - 添加 PointCloud2 显示，查看 `obstacle` 话题
   - 添加 Bool 显示，查看 `/match` 和 `/diverge` 话题

### 调试模式

设置 `debug_en=true` 后，系统会输出详细的处理时间信息和障碍物点百分比。

### 启用配准

设置 `align_en=true` 后，系统会使用 GICP 进行点云配准，提高检测精度。注意这会增加计算时间。

## 代码重构说明

本次重构的主要改进：

1. **代码清理**:
   - 删除所有注释掉的代码（Livox_Scan_Callback, hole_path 相关代码等）
   - 移除未使用的变量（`hole`, `costmap_points`, `flat_kdtree`, `min_diverge_num`, `T_baselink_sensor`, `costmap`, `costmap_distance_threshold` 等）
   - 移除未使用的函数（`grid2pointcloud`, `Costmap_Callback`, `transform` 等）
   - 清理重复的头文件包含

2. **代码规范**:
   - 统一命名规范（成员变量使用下划线后缀）
   - 添加详细的注释和文档
   - 改进代码结构和可读性
   - 将调试输出改为使用 ROS 日志系统

3. **功能保持**:
   - 所有原有功能完全保留
   - 算法逻辑与重构前完全一致
   - 接口兼容性保持不变

4. **代码组织**:
   - 按功能分组组织成员变量
   - 改进错误处理和日志输出
   - 统一代码风格

## 故障排除

### 常见问题

1. **PCL 版本不匹配**
   - 确保安装了 PCL 1.11
   - 检查 CMakeLists.txt 中的 PCL 路径是否正确

2. **检测不到障碍物**
   - 检查 `distance_threshold` 参数是否合适
   - 检查地图文件是否正确加载
   - 检查点云话题是否正确发布

3. **匹配失败**
   - 增大 `diverge_threshold` 参数
   - 检查地图和当前环境是否匹配
   - 尝试启用配准（`align_en=true`）

4. **处理速度慢**
   - 增大 `leaf_size` 参数
   - 减小 `box_size` 参数
   - 禁用配准（`align_en=false`）

### 调试技巧

1. 启用调试模式查看处理时间：
```bash
rosparam set /detector/debug_en true
```

2. 查看匹配状态：
```bash
rostopic echo /match
rostopic echo /diverge
```

3. 检查障碍物点云：
```bash
rostopic echo /obstacle --noarr
```

## 性能优化建议

1. **点云滤波**: 适当增大 `leaf_size` 可以减少点云数量，提高处理速度
2. **裁剪区域**: 根据实际需求设置 `box_size`，平衡处理速度和检测范围
3. **配准**: 在保证精度的前提下，可以禁用配准以提高速度
4. **频率**: 根据实际需求设置 `freq`，平衡实时性和计算负载

## 许可证

TODO

## 维护者

wqx

## 更新日志

### 重构版本
- 删除未使用的代码和变量
- 添加详细注释和文档
- 统一代码风格和命名规范
- 改进错误处理和日志输出
- 保持功能完全一致


