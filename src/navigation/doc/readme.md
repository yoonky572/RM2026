
# Navigation 模块文档

## 1. 模块概述

`navigation` 模块为移动机器人提供从全局路径到速度控制的完整闭环。核心目标是在确保安全性的前提下，使机器人沿平滑路径行驶至目标点，同时在定位不稳定或环境复杂时具备自检与降级能力。

## 2. 目录结构

```
src/navigation/
├── include/navigation/
│   ├── controller.hpp          # 控制器接口与私有辅助函数声明
│   ├── fieldoptimizer.hpp      # 势场优化器接口
│   └── pid.hpp                 # PID 控制器接口
├── src/
│   ├── controller.cpp          # 控制器核心逻辑
│   ├── fieldoptimizer.cpp      # 势场优化实现
│   ├── pid.cpp                 # PID 占位实现
│   └── costmap_clean.cpp       # 代价地图清理节点
├── launch/
│   └── controller.launch       # 控制器示例启动文件
└── doc/readme.md               # 模块说明文档（本文件）
```

## 3. 核心功能

| 组件 | 功能 |
|------|------|
| `Controller` | 管理 ROS 话题、定时器和状态机，周期性执行规划与优化循环，最终发布 `/cmd_vel` |
| `Field Optimizer` | 基于人工势场对剪枝后的路径点进行局部优化，远离障碍物并保持平滑 |
| `Costmap Clean` | 定时调用 `/move_base/clear_costmaps`，保持代价地图干净 |
| `PID` | 预留的 PID 控制器接口，待根据需求补充 `calculate()` 实现 |

## 4. Controller 模块详解

### 4.1 初始化阶段
- `loadParameters()`：加载运动、规划、优化、恢复行为、调试等参数。
- `initializePublishers()`：创建 `cmd_vel`、`prune_path`、`local_path`、`obstacle_grids` 等发布器。
- `initializeSubscribers()`：订阅全局路径、代价地图、定位/匹配状态、move_base 状态等话题。
- `initializeTimers()`：创建 `Plan`（规划循环）与 `PathOptimaze`（路径优化循环）两个定时器。

### 4.2 Plan 循环
1. **状态判断**：检查是否已有规划、定位是否稳定、是否已到达目标。
2. **路径处理**：从优化路径中提取当前跟随段，计算曲率与前进方向。
3. **控制策略**：
   - 航向误差过大时先调用 `handleTurningInPlace()` 原地转向。
   - `FollowTraj()` 输出基础速度命令，并在 `applySpeedScalingNearGoal()` 中接近目标时减速。
4. **安全降级**：定位未就绪或缺少有效路径时，调用 `publishStopCommand()` 输出不同状态标志。

### 4.3 PathOptimaze 循环
1. **剪枝**：`extractPrunePathFromGlobal()` 仅保留代价地图中 cost=0 的路径点。
2. **狭窄检测**：`detectNarrowPassage()` + `updateNarrowPassageStatus()` 判断是否处于狭窄通道。
3. **势场优化**：`optimizePathPoints()` 结合障碍物斥力和目标引力对前若干点进行优化。
4. **平滑处理**：`removeClosePathPoints()` 去除距离过近的点，减少路径抖动。
5. **发布**：更新并发布剪枝路径、局部路径，同时重置 `follow_index`。

## 5. Field Optimizer

- `force_calculate()`：计算单个障碍物产生的斥力。
- `attractive_calculate()`：计算目标点的引力，避免偏离原始路径。
- `optimize()`：迭代叠加力并更新点位置，满足步长收敛或达到最大迭代次数。
- `ratio` 参数用于限制优化幅度，确保路径不会被过度推离。

## 6. Baseline 行为

- **路径跟踪**：默认前视索引 10，纯追踪控制，靠近目标时线性降速。
- **状态管理**：`linear.z` 用作状态标志位（0=未定位/等待，1=无路径但定位成功，>1 表示其他状态）。
- **安全逻辑**：航向误差大于阈值先转向；定位未稳定时保持停止；全局规划失败时输出停机命令。
- **优化策略**：在狭窄通道或 hole 模式下可跳过优化以节省计算。

## 7. 使用说明

1. 启动 `roslaunch navigation controller.launch`（或自行编写 launch，确保提供必要话题）。
2. 根据需求调整 `param` 文件中的 `plan_frequency`、`p_value`、`narrow_threshold` 等参数。
3. 需要定期清理代价地图时，可单独运行 `rosrun navigation costmap_clean`.
4. 若要使用 PID 控制，请在 `pid.cpp` 中完善 `calculate()`，并在控制流程中接入。

## 8. FAQ

- **为什么 `FindNearstPose` 需要非 const 引用？**  
  函数内部会对传入的 Pose 进行更新，需要可写引用；若源数据是 const，需要先复制。

- **为何使用 `cmd_vel.linear.z` 表示状态？**  
  项目既有约定，便于上层或底盘节点监听控制器状态。

- **Plan 与 PathOptimaze 的频率如何配置？**  
  通过参数 `plan_frequency`、`opt_freq` 设置，默认分别为 30Hz 与 3Hz，互不干扰。

---

如需扩展（例如加入恢复行为、完成 PID、支持动态参数等），建议沿用当前的函数拆分方式，保持模块化与可维护性。

## 9. 模块架构（简版）

- **输入**：全局路径、全局代价地图、定位/匹配状态、move_base 状态。
- **Controller 内部**
  1. `Plan` 循环：负责目标判定、路径跟踪、速度发布。
  2. `PathOptimaze` 循环：负责剪枝、狭窄检测、势场优化。
  3. 状态管理：维护定位状态、规划状态、跟踪索引等。
- **输出**：剪枝路径 `/prune_path`、局部路径 `/local_path`、障碍物栅格 `/obstacle_grids`、速度指令 `/cmd_vel`。
- **Field Optimizer**：由 `PathOptimaze` 调用，计算障碍物斥力 + 目标引力，输出优化后的路径点。

## 10. 控制流程（文本版）

1. **数据更新**：持续接收全局路径与代价地图；定位/匹配状态驱动 `plan/localized` 标志。
2. **PathOptimaze 周期**  
   a. 在全局路径上向前扫描，保留 cost=0 的点形成剪枝路径。  
   b. 统计周围障碍物数量，判断是否为狭窄通道。  
   c. 调用势场优化器对前 `short_forsee_index` 个点优化并去除过近的点。  
   d. 发布最新的剪枝/局部路径，重置跟随索引。  
3. **Plan 周期**  
   a. 读取机器人位姿、计算与目标距离。  
   b. 如果到达目标则输出成功状态；若定位未稳定或缺少路径则保持停止。  
   c. 从优化路径中提取跟随段，判断是否需要原地转向。  
   d. 执行轨迹跟随，并在接近目标时降速，最终发布 `/cmd_vel`。

## 11. Baseline 行为 Matrices

### 11.1 状态机

| 状态 | 触发条件 | 输出行为 |
|------|----------|----------|
| `WAITING_LOCALIZATION` | `localized=false` 或 `定位成功 <5s` | `cmd_vel.linear.z=0`，保持静止 |
| `NO_PATH` | 已定位但 `prune_path`/`opt_path` 为空 | `cmd_vel.linear.z=1`，提示“NO PATH” |
| `TURN_IN_PLACE` | `|yaw_error|` 超阈值且距离>1m | Z 轴角速度由 `YawControl` 决定 |
| `FOLLOW_PATH` | 有效路径且姿态误差可接受 | `FollowTraj` 输出速度，接近目标时降速 |
| `GOAL_REACHED` | 距离≤`goal_dist_tolerance` | 线速度=0，`linear.z=1` 表示 success |

### 11.2 参数基线

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `plan_frequency` | 30 Hz | 规划循环频率 |
| `opt_freq` | 3 Hz | 路径优化频率 |
| `goal_dist_tolerance` | 0.2 m | 判定到达的距离阈值 |
| `FORESEE_POSE_INDEX` | 10 | 轨迹前视点索引 |
| `MIN_POINT_DISTANCE` | 0.05 m | 优化后相邻点最小距离 |
| `narrow_threshold` | 5 | 判定狭窄通道的障碍物数 |
| `NEAR_GOAL_DISTANCE` | 1.0 m | 进入减速区的距离 |

## 12. 数据流

1. **输入**：`/move_base/GlobalPlanner/plan`、`/move_base/global_costmap/costmap`、`/match`、`/localize_success`、`/move_base/status`
2. **中间数据**：
   - `global_path`（原始） → `prune_path`（剪枝） → `opt_path`（优化）
   - `obstacle_result`：`Passbility_check` 输出，用于势场优化与狭窄判定
3. **输出**：`/prune_path`、`/local_path`、`/obstacle_grids`、`/cmd_vel`

## 13. 关键函数速览

| 函数 | 作用 |
|------|------|
| `Plan()` | 主规划循环，协调目标检测、路径跟踪与速度发布 |
| `isGoalReached()` | 根据距离和路径索引判断是否到达目标 |
| `canExecutePath()` | 检查剪枝/优化路径是否可用以及全局规划器状态 |
| `extractFollowingPath()` | 从优化路径截取当前跟随段；内部依赖 `FindNearstPose()` 锁定最近点 |
| `shouldTurnInPlace()` / `handleTurningInPlace()` | 判断并执行原地转向，解决大航向误差 |
| `handlePathTracking()` | 调用 `FollowTraj()` 生成速度命令，并通过 `applySpeedScalingNearGoal()` 接近目标时减速 |
| `PathOptimaze()` | 路径优化循环，负责剪枝、狭窄检测、势场优化 |
| `extractPrunePathFromGlobal()` | 使用 `NavigationUtils::world2Grid()` 过滤 cost=0 的全局路径点 |
| `detectNarrowPassage()` | 调用 `Passbility_check()` 统计障碍物并更新狭窄状态 |
| `optimizePathPoints()` | 组合 `Field_Optimizer::optimize()` 生成更安全的局部路径 |
| `removeClosePathPoints()` | 删除距离过近的点，保持路径平滑 |
| `NavigationUtils::world2Grid()/Grid2world()` | 世界-栅格坐标转换；被剪枝、障碍物检测等流程复用 |
| `Field_Optimizer::optimize()` | 势场迭代核心，叠加斥力/引力并控制步长收敛 |

## 14. 扩展建议

- **PID 完成实现**：在 `pid.cpp` 中补充 `calculate()`，并在 `FollowTraj` 或姿态控制中引入 PID 输出，以提高速度/姿态稳定性。
- **动态参数调整**：可接入 `dynamic_reconfigure`，在运行时调整 `goal_dist_tolerance`、`wz_p_value` 等。
- **恢复行为**：结合 `min_recover_radius/max_recover_radius`，实现自由空间搜索（`FindNearstFreeSpace` 已提供基础函数）。

---

该 README 现已覆盖功能说明、架构图、流程图、baseline 参数及扩展指南，便于团队成员快速理解与维护。

