# Navigation 模块文档

## 1. 模块概述

`navigation` 模块为移动机器人提供简单的路径跟踪控制。核心功能是根据全局路径规划器提供的路径，使机器人沿平滑轨迹行驶至目标点。

## 2. 目录结构

```
src/navigation/
├── include/navigation/
│   ├── controller.hpp          # 控制器接口声明
│   └── utility.hpp             # 工具函数（距离计算、坐标转换等）
├── include/cubic_spline/       # 三次样条插值库（用于路径平滑）
├── src/
│   └── controller.cpp          # 控制器核心逻辑
├── launch/
│   └── controller.launch       # 控制器启动文件
└── doc/readme.md               # 模块说明文档（本文件）
```

## 3. 核心功能

| 组件 | 功能 |
|------|------|
| `Controller` | 管理 ROS 话题、定时器，周期性执行规划循环，最终发布 `/cmd_vel` |
| `GenTraj` | 使用三次样条插值生成平滑的局部轨迹 |
| `FollowTraj` | 基于位置跟踪的路径跟随控制 |

## 4. Controller 模块详解

### 4.1 初始化阶段

在构造函数中完成：
- **参数加载**：从ROS参数服务器加载运动控制、规划等参数
- **发布器初始化**：创建 `local_path`、`cmd_vel` 等话题发布器
- **订阅器初始化**：订阅全局路径 `/move_base/GlobalPlanner/plan`、定位状态 `/diverge`
- **定时器初始化**：创建 `Plan` 定时器，按设定频率执行规划循环

### 4.2 Plan 循环

主控制循环，按 `plan_frequency` 频率执行：

1. **状态检查**：
   - 检查是否有有效规划（`plan` 标志）
   - 无规划时，发布停止命令（可设置 `set_yaw_speed` 原地旋转）

2. **目标检测**：
   - 获取机器人当前位姿
   - 计算到目标点的距离
   - 如果距离小于 `goal_dist_tolerance` 或到达路径末端，判定到达目标

3. **路径处理**：
   - 调用 `FindNearstPose()` 在全局路径上找到距离机器人最近的路径点
   - 从全局路径中截取一段局部路径（当前点往后20个点）
   - 调用 `GenTraj()` 使用三次样条插值生成平滑的局部轨迹

4. **速度控制**：
   - 调用 `FollowTraj()` 根据局部轨迹计算机器人速度指令
   - 发布 `/cmd_vel` 速度指令

### 4.3 核心函数

#### FindNearstPose()
在全局路径上找到距离机器人最近的路径点索引。从当前索引开始向前搜索，当距离开始增加时停止，找到最近点。

#### FollowTraj()
路径跟踪控制函数：
- 计算机器人到局部路径第一个目标点的方向角
- 根据距离和方向计算全局坐标系下的速度
- 限制最大速度不超过 `max_speed`
- 将全局速度转换到机器人坐标系并发布

## 5. 参数说明

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `max_speed` | double | 1.0 | 最大线速度（m/s） |
| `p_value` | double | 0.5 | 速度比例系数 |
| `set_yaw_speed` | double | 0 | 无规划时的角速度（rad/s） |
| `plan_frequency` | int | 30 | 规划循环频率（Hz） |
| `goal_dist_tolerance` | double | 0.2 | 到达目标的距离阈值（m） |
| `prune_ahead_distance` | double | 0.5 | 前视距离阈值（m） |
| `global_frame` | string | "map" | 全局坐标系名称 |

## 6. 话题说明

### 订阅的话题

| 话题名 | 类型 | 说明 |
|--------|------|------|
| `/move_base/GlobalPlanner/plan` | `nav_msgs/Path` | 全局规划器生成的路径 |
| `/diverge` | `std_msgs/Bool` | 定位发散状态（true=发散，停止规划） |

### 发布的话题

| 话题名 | 类型 | 说明 |
|--------|------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | 速度控制指令 |
| `/local_path` | `nav_msgs/Path` | 局部平滑轨迹（用于可视化） |

## 7. 控制流程

1. **接收全局路径**：当接收到新的全局路径时，启用路径跟踪（`plan = true`）
2. **定位状态监控**：如果定位发散（`/diverge` 为 true），停止路径跟踪
3. **规划循环**：
   - 检查是否到达目标
   - 找到最近路径点
   - 提取局部路径（20个点）
   - 使用三次样条生成平滑轨迹
   - 执行路径跟踪并发布速度指令
4. **到达目标**：距离小于阈值时，发布停止命令，重置索引

## 8. 使用说明

### 8.1 启动控制器

```bash
roslaunch navigation controller.launch
```

或根据实际需求配置 launch 文件，确保提供以下必要话题：
- `/move_base/GlobalPlanner/plan` - 全局路径
- `/diverge` - 定位状态（可选）

### 8.2 参数调整

在 launch 文件中通过 `<param>` 标签设置参数，例如：

```xml
<node name="controller" pkg="navigation" type="controller">
    <param name="max_speed" value="1.0"/>
    <param name="p_value" value="0.5"/>
    <param name="plan_frequency" value="30"/>
    <param name="goal_dist_tolerance" value="0.2"/>
</node>
```

### 8.3 可视化

- 发布局部路径 `/local_path` 可在 RViz 中可视化
- 速度指令 `/cmd_vel` 发送到底盘控制节点

## 9. 算法说明

### 9.1 路径平滑

使用三次样条插值（`GenTraj`）将离散的路径点转换为平滑的轨迹，确保机器人运动的连续性和平滑性。

### 9.2 速度控制

采用位置跟踪方式：
- 速度大小与到下一个路径点的距离成正比：`v = distance * p_value`
- 速度方向指向下一个路径点
- 限制最大速度不超过 `max_speed`

### 9.3 坐标系转换

- 全局路径在 `global_frame` 坐标系（通常为 "map"）
- 速度指令在机器人坐标系（`base_link`）
- 通过机器人当前航向角进行坐标系转换

## 10. FAQ

**Q: 为什么路径需要平滑处理？**  
A: 原始路径点可能间距较大或不平滑，直接跟踪可能导致机器人运动不连续。通过三次样条插值可以生成平滑轨迹，提高跟踪质量。

**Q: 如何调整机器人的跟踪速度？**  
A: 调整 `p_value` 参数可以改变速度响应灵敏度，增大该值会提高速度响应，但可能影响稳定性。

**Q: 定位发散时会发生什么？**  
A: 当 `/diverge` 话题为 true 时，控制器会停止路径跟踪（`plan = false`），机器人将保持停止或按照 `set_yaw_speed` 原地旋转。

**Q: 如何判断是否到达目标？**  
A: 当机器人到目标点的距离小于 `goal_dist_tolerance`，或者到达全局路径的最后一个点时，判定为到达目标。

---

该模块采用简洁的设计，专注于核心的路径跟踪功能，便于理解、维护和扩展。
