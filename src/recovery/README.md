# Recovery 恢复行为包

## 概述

本包提供了两个自定义的 ROS 恢复行为插件，用于处理导航中的异常情况：

1. **Goal_Reset**: 当目标点位于障碍物中时，自动寻找最近的自由空间并重置目标点
2. **FindFreespace**: 当机器人被困在障碍物中时，寻找自由空间并发布速度命令使机器人逃离

这两个恢复行为都继承自 `nav_core::RecoveryBehavior`，可以作为 move_base 的恢复行为插件使用。

## 功能特性

### Goal_Reset（目标重置）

- **功能**: 监听目标点话题，当检测到目标点在障碍物中时，自动在搜索半径内寻找自由空间
- **策略**: 计算障碍物中心，选择距离障碍物中心最远的自由空间点作为新目标
- **输出**: 发布新的目标点到 `/move_base_simple/goal` 话题，并发布可视化标记

### FindFreespace（寻找自由空间）

- **功能**: 检测机器人是否被困在障碍物中，如果是则寻找自由空间并发布速度命令
- **策略**: 检查机器人周围四个方向是否都是自由空间，如果不是则搜索最近的自由空间点
- **输出**: 发布速度命令到 `/escape_vel` 话题，并发布可视化标记

## 代码结构

```
recovery/
├── CMakeLists.txt              # 构建配置
├── package.xml                  # 包配置
├── custom_recovery.xml          # 插件注册文件
├── README.md                   # 本文档
├── include/
│   ├── recovery/
│   │   ├── goal_reset.hpp      # Goal_Reset 类头文件
│   │   └── find_freespace.hpp  # FindFreespace 类头文件
│   └── util/
│       ├── costmap_utils.hpp    # 代价地图工具函数（坐标转换）
│       └── freespace_finder.hpp # 自由空间搜索工具函数
└── src/
    ├── goal_reset.cpp          # Goal_Reset 类实现
    └── find_freespace.cpp      # FindFreespace 类实现
```

## 依赖项

### ROS 包依赖
- `roscpp`: ROS C++ 客户端库
- `pluginlib`: 插件库
- `nav_core`: 导航核心接口
- `costmap_2d`: 代价地图
- `geometry_msgs`: 几何消息类型
- `visualization_msgs`: 可视化消息类型
- `tf2` / `tf2_ros`: 坐标变换库

### 系统依赖
- C++17 编译器
- Boost 库

## 编译

```bash
cd ~/catkin_ws
catkin_make
# 或
catkin build recovery
```

## 配置

### 在 move_base 中使用

在 `move_base` 的配置文件中添加恢复行为：

```yaml
recovery_behaviors:
  - name: 'recovery/Goal_Reset'
    type: 'recovery/Goal_Reset'
  - name: 'recovery/FindFreespace'
    type: 'recovery/FindFreespace'
```

### 参数配置

#### Goal_Reset 参数

- `max_search_radius` (double, 默认: 0.3): 最大搜索半径（米）

在 launch 文件或参数服务器中设置：

```xml
<rosparam>
  recovery/Goal_Reset:
    max_search_radius: 0.3
</rosparam>
```

#### FindFreespace 参数

- `max_search_radius` (double, 默认: 0.3): 最大搜索半径（米）
- `speed` (double, 默认: 0.5): 逃离速度（米/秒）

在 launch 文件或参数服务器中设置：

```xml
<rosparam>
  recovery/FindFreespace:
    max_search_radius: 0.3
    speed: 0.5
</rosparam>
```

## 话题

### Goal_Reset

#### 订阅
- `/move_base_simple/goal` (geometry_msgs/PoseStamped): 目标点输入

#### 发布
- `/move_base_simple/goal` (geometry_msgs/PoseStamped): 重置后的目标点
- `new_goal` (visualization_msgs/Marker): 新目标点的可视化标记

### FindFreespace

#### 发布
- `/escape_vel` (geometry_msgs/Twist): 逃离速度命令
- `escape_goal` (visualization_msgs/Marker): 逃离目标点的可视化标记

## 算法说明

### 自由空间搜索算法

两个恢复行为都使用相同的自由空间搜索策略：

1. **坐标转换**: 将世界坐标转换为栅格坐标
2. **逐层搜索**: 从中心点开始，逐层向外搜索（半径从 0 到 max_search_radius）
3. **分类收集**: 同时收集自由空间点和障碍物点
4. **障碍物中心计算**: 计算所有障碍物点的中心坐标
5. **最优选择**: 选择距离障碍物中心最远的自由空间点作为目标

### 坐标转换

使用标准的代价地图坐标转换方法：
- **世界坐标 → 栅格坐标**: `grid = (world - origin) / resolution`
- **栅格坐标 → 世界坐标**: `world = (grid + 0.5) * resolution + origin`

### 速度命令计算（FindFreespace）

1. 计算目标方向（全局坐标系）
2. 获取机器人当前朝向
3. 计算全局坐标系下的速度分量
4. 转换到机器人局部坐标系
5. 持续发布速度命令（默认 1 秒）

## 使用示例

### 在 RViz 中可视化

1. 启动 move_base 和恢复行为
2. 在 RViz 中添加 Marker 显示：
   - `new_goal` (红色球体，表示 Goal_Reset 的新目标)
   - `escape_goal` (蓝色球体，表示 FindFreespace 的逃离目标)

### 测试 Goal_Reset

1. 在 RViz 中设置一个位于障碍物中的目标点
2. Goal_Reset 会自动检测并重置目标点到最近的自由空间
3. 观察红色标记显示的新目标点位置

### 测试 FindFreespace

1. 将机器人移动到障碍物附近或内部
2. FindFreespace 会检测机器人是否被困
3. 如果被困，会发布速度命令使机器人逃离
4. 观察蓝色标记显示的逃离目标点

## 代码重构说明

本次重构的主要改进：

1. **代码复用**: 提取公共工具函数到 `util` 命名空间
   - `costmap_utils.hpp`: 坐标转换函数
   - `freespace_finder.hpp`: 自由空间搜索和选择函数

2. **代码清理**:
   - 删除未使用的代码和变量（如 `min_r`、空的 `freespacesearch.hpp`）
   - 移除未使用的函数声明（如 `FindFreespace::GoalCallback`）
   - 清理调试信息，使用适当的日志级别

3. **代码规范**:
   - 统一命名规范（成员变量使用下划线后缀）
   - 添加详细的注释和文档
   - 改进代码结构和可读性

4. **功能保持**:
   - 所有原有功能完全保留
   - 行为逻辑与重构前完全一致
   - 接口兼容性保持不变

## 故障排除

### 常见问题

1. **插件未加载**
   - 检查 `custom_recovery.xml` 中的库路径是否正确
   - 确认库文件已正确编译到 `lib/` 目录

2. **找不到自由空间**
   - 增大 `max_search_radius` 参数
   - 检查代价地图是否正确更新

3. **速度命令无效**
   - 检查 `/escape_vel` 话题是否被正确订阅
   - 确认机器人控制器正在监听该话题

## 许可证

TODO

## 维护者

tjurm

## 更新日志

### 重构版本
- 提取公共工具函数，提高代码复用性
- 添加详细注释和文档
- 清理未使用的代码
- 统一代码风格和命名规范
- 保持功能完全一致


