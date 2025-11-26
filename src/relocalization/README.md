# Relocalization 重定位包

## 概述

本包实现了一个基于点云配准的机器人重定位系统，使用 GICP (Generalized Iterative Closest Point) 算法进行精确配准，并在定位失败时使用 FPFH 特征进行粗配准。

## 功能特性

- **点云配准**: 使用 small_gicp 库实现高效的 GICP 配准
- **粗配准**: 当定位失败时，使用 FPFH 特征和 SAC-IA 进行粗配准
- **实时定位**: 支持实时处理激光雷达扫描和障碍物点云
- **TF 发布**: 自动发布 `vice_map` 到 `odom` 的 TF 变换
- **调试支持**: 可选的点云可视化用于调试

## 代码结构

```
relocalization/
├── CMakeLists.txt              # 构建配置
├── package.xml                  # 包配置
├── README.md                   # 本文档
├── launch/
│   └── relocalization.launch   # 启动文件
├── include/
│   └── relocalization/
│       └── relocalization.hpp  # 类头文件
└── src/
    └── relocalization.cpp      # 类实现
```

## 依赖项

### ROS 包依赖
- `roscpp`: ROS C++ 客户端库
- `tf2_ros`: TF2 库
- `sensor_msgs`: 传感器消息类型
- `geometry_msgs`: 几何消息类型
- `std_msgs`: 标准消息类型

### 第三方库依赖
- **PCL 1.11.0**: 点云处理库
  - `pcl_common`
  - `pcl_kdtree`
  - `pcl_io`
  - `pcl_filters`
  - `pcl_features`
  - `pcl_search`
  - `pcl_sample_consensus`
  - `pcl_octree`
- **Eigen3**: 线性代数库
- **OpenMP**: 并行计算库
- **small_gicp**: 快速 GICP 实现（包含在包中）

## 编译

```bash
cd ~/catkin_ws
catkin_make
# 或
catkin build relocalization
```

**注意**: 本包需要特定版本的 PCL 1.11.0，CMakeLists.txt 中已配置了自定义的 PCL 路径。请根据您的系统环境调整路径。

## 配置

### 启动文件参数

在 `launch/relocalization.launch` 中配置以下参数：

#### 必需参数
- `pcd_path` (string): 先验地图 PCD 文件路径
- `obs_path` (string): 先验障碍物点云 PCD 文件路径

#### 配准参数
- `leaf_size` (double, 默认: 0.1): 扫描点云体素滤波的叶子大小（米）
- `map_leaf_size` (double, 默认: 0.5): 地图点云体素滤波的叶子大小（米）
- `max_dist_sq` (double, 默认: 1.0): 配准中最大距离平方阈值
- `max_iterations` (double, 默认: 100): 配准最大迭代次数
- `num_neighbors` (int, 默认: 10): 协方差估计的最近邻数量
- `num_threads` (int, 默认: 4): 并行计算的线程数

#### 定位参数
- `diverge_threshold` (double, 默认: 10.0): 配准误差阈值，低于此值认为定位成功
- `freq` (int, 默认: 10): 配准运行频率（Hz）
- `max_frame` (int, 默认: 10): 点云队列最大缓存帧数

#### 调试参数
- `debug_en` (bool, 默认: false): 是否启用调试模式（发布可视化点云）

### 示例配置

```xml
<node name="relocalization" pkg="relocalization" type="relocalization" output="screen">
    <param name="pcd_path" value="/path/to/map.pcd"/>
    <param name="obs_path" value="/path/to/obstacles.pcd"/>
    <param name="leaf_size" value="0.2"/>
    <param name="map_leaf_size" value="0.2"/>
    <param name="max_dist_sq" value="2.0"/>
    <param name="num_threads" value="8"/>
    <param name="max_iterations" value="20"/>
    <param name="diverge_threshold" value="120.0"/>
    <param name="debug_en" value="true"/>
    <param name="freq" value="5"/>
    <param name="max_frame" value="1"/>
</node>
```

## 话题

### 订阅

- `/ground_segmentation/obstacle_cloud` (sensor_msgs/PointCloud2): 障碍物点云
- `/livox/lidar_ros` (sensor_msgs/PointCloud2): 激光雷达扫描点云
- `/initialpose` (geometry_msgs/PoseWithCovarianceStamped): 初始位姿（用于手动设置初始位置）
- `/diverge` (std_msgs/Bool): 发散标志（当检测到定位发散时设置为 true）
- `/body_pose` (geometry_msgs/PoseStamped): 机器人位姿（用于更新配准初始值）

### 发布

- `/localize_success` (std_msgs/Bool): 定位成功标志
- `target` (sensor_msgs/PointCloud2): 目标点云（调试用，仅在 `debug_en=true` 时发布）
- `source` (sensor_msgs/PointCloud2): 源点云（调试用，仅在 `debug_en=true` 时发布）
- `aligned` (sensor_msgs/PointCloud2): 配准后的点云（调试用，仅在 `debug_en=true` 时发布）

### TF 发布

- `vice_map` → `odom`: 发布重定位后的变换

## 算法说明

### 配准流程

1. **点云预处理**:
   - 对障碍物点云进行统计离群点去除
   - 对扫描点云和障碍物点云进行体素滤波
   - 生成协方差点云用于 GICP 配准

2. **配准执行**:
   - 如果定位丢失 (`lost_=true`): 使用 FPFH 特征和 SAC-IA 进行粗配准，然后使用粗配准结果作为初始值进行 GICP 配准
   - 如果定位未丢失: 使用上一次的配准结果作为初始值进行 GICP 配准

3. **结果判断**:
   - 如果配准误差小于阈值: 定位成功，重置相关标志
   - 如果配准误差大于阈值: 定位失败，如果误差没有改善，尝试调整初始位姿（旋转偏航角）

### FPFH 粗配准

当定位丢失时，使用以下步骤进行粗配准：

1. 计算源点云和目标点云的法向量
2. 基于法向量计算 FPFH (Fast Point Feature Histograms) 特征
3. 使用 SAC-IA (Sample Consensus Initial Alignment) 算法进行初始配准
4. 将粗配准结果作为 GICP 的初始值

### 偏航角调整策略

当配准无法收敛时，系统会尝试调整初始位姿：

- 交替尝试正负方向的偏航角旋转
- 每次旋转角度递增（约 20 度）
- 考虑俯仰角的影响，计算旋转轴

## 使用示例

### 基本使用

1. 准备地图文件：
   - 生成先验地图 PCD 文件
   - 生成障碍物点云 PCD 文件

2. 启动节点：
```bash
roslaunch relocalization relocalization.launch
```

3. 在 RViz 中可视化：
   - 添加 TF 显示，查看 `vice_map` 到 `odom` 的变换
   - 如果启用调试模式，添加 PointCloud2 显示，查看 `target`、`source`、`aligned` 话题

### 手动设置初始位姿

在 RViz 中使用 "2D Pose Estimate" 工具设置初始位姿，系统会订阅 `/initialpose` 话题并更新配准初始值。

### 调试模式

设置 `debug_en=true` 后，系统会发布以下点云用于调试：
- `target`: 滤波后的先验地图
- `source`: 转换到 odom 坐标系的当前扫描
- `aligned`: 配准后的点云

## 代码重构说明

本次重构的主要改进：

1. **代码清理**:
   - 删除未使用的 `Registration` 类
   - 删除所有注释掉的代码
   - 移除未使用的变量（`use_stl_cloud`, `precise_diverge_threshold`, `y_dist`, `high_features_map`, `high_features_scan`, `last_diverge_data`, `match` 等）
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
   - 提取公共功能到独立函数（`publishDebugClouds`, `publishTransform`）
   - 改进错误处理和日志输出

## 故障排除

### 常见问题

1. **PCL 版本不匹配**
   - 确保安装了 PCL 1.11.0
   - 检查 CMakeLists.txt 中的 PCL 路径是否正确

2. **定位失败**
   - 检查地图文件是否正确加载
   - 增大 `diverge_threshold` 参数
   - 检查点云话题是否正确发布
   - 尝试手动设置初始位姿

3. **TF 变换未发布**
   - 检查定位是否成功（查看 `/localize_success` 话题）
   - 确认 `vice_map` 坐标系已正确设置

4. **配准速度慢**
   - 增大 `leaf_size` 和 `map_leaf_size` 参数
   - 减少 `max_iterations` 参数
   - 增加 `num_threads` 参数（如果 CPU 核心数足够）

### 调试技巧

1. 启用调试模式查看点云：
```bash
rosparam set /relocalization/debug_en true
```

2. 查看配准误差：
```bash
rostopic echo /localize_success
```

3. 检查 TF 变换：
```bash
rosrun tf tf_echo vice_map odom
```

## 性能优化建议

1. **点云滤波**: 适当增大 `leaf_size` 可以减少点云数量，提高配准速度
2. **线程数**: 根据 CPU 核心数设置 `num_threads`
3. **迭代次数**: 在保证精度的前提下，减少 `max_iterations` 可以提高速度
4. **缓存帧数**: 减少 `max_frame` 可以减少内存使用

## 许可证

TODO

## 维护者

wqx

## 更新日志

### 重构版本
- 删除未使用的代码和类
- 添加详细注释和文档
- 统一代码风格和命名规范
- 改进错误处理和日志输出
- 保持功能完全一致


