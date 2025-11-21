# RoboMaster 哨兵决策框架

这是一个优雅且可扩展的RoboMaster哨兵决策框架，根据游戏状态（血量、弹丸、建筑状态等）智能规划下一步的目标位置。

## 架构设计

### 核心组件

1. **GameState（游戏状态管理）**
   - 管理自身血量、弹丸数量
   - 跟踪建筑血量（前哨站、基地）
   - 监控敌人状态和位置
   - 提供高级状态判断（是否低血量、是否需要撤退等）

2. **GoalManager（目标点管理器）**
   - 管理所有可用的目标点位置
   - 支持动态添加、删除、更新目标点
   - 支持优先级排序和启用/禁用
   - 从ROS参数加载目标点配置

3. **DecisionEngine（决策引擎）**
   - 基于游戏状态做出最优决策
   - 支持多种策略：防御、攻击、平衡、撤退、重新定位
   - 可配置的策略权重
   - 返回决策结果和置信度

4. **SentryDecisionNode（决策节点）**
   - ROS节点，集成所有组件
   - 订阅游戏状态话题
   - 发布目标点到导航系统

## 使用方法

### 1. 编译

```bash
cd ~/Code/Sentry25NAVI
catkin_make
```

### 2. 运行决策节点

```bash
roslaunch goal_distribute sentry_decision.launch
```

### 3. 配置参数

在launch文件中可以配置以下参数：

- `decision_rate`: 决策频率（Hz），默认10.0
- `default_strategy`: 默认策略
  - 0 = DEFENSIVE（防御）
  - 1 = AGGRESSIVE（攻击）
  - 2 = BALANCED（平衡，默认）
  - 3 = RETREAT（撤退）
  - 4 = REPOSITION（重新定位）
- `team_color`: 队伍颜色（"red"或"blue"）
- `initial_hp`: 初始血量，默认600
- `initial_ammo`: 初始弹丸数，默认500

### 4. ROS话题接口

#### 订阅话题（接收游戏状态）

- `/game_state/self_hp` (std_msgs/Int32): 自身血量
- `/game_state/self_ammo` (std_msgs/Int32): 自身弹丸数量
- `/game_state/building_hp` (std_msgs/Int32): 建筑血量（待扩展消息格式）
- `/game_state/enemy` (geometry_msgs/PoseStamped): 敌人位置

#### 发布话题

- `/move_base_simple/goal` (geometry_msgs/PoseStamped): 导航目标点
- `/integer_topic` (std_msgs/Int32): 目标点ID
- `/sentry_strategy` (std_msgs/String): 当前决策策略原因

## 决策策略说明

### 防御策略（DEFENSIVE）
- 当基地或前哨站血量低于阈值时，优先移动到保护建筑的位置
- 选择高优先级防御性目标点

### 攻击策略（AGGRESSIVE）
- 当敌人可见时，选择接近敌人的攻击位置
- 优先选择前沿阵地

### 平衡策略（BALANCED）
- 综合考虑血量、弹丸、建筑状态、敌人位置等因素
- 使用加权评分选择最佳目标点

### 撤退策略（RETREAT）
- 当自身低血量或建筑处于危险时自动触发
- 选择安全的、靠近基地的位置
- 远离敌人

### 重新定位策略（REPOSITION）
- 根据当前状态重新评估位置
- 选择评分最高的目标点

## 扩展指南

### 添加新的决策策略

1. 在`DecisionEngine::Strategy`枚举中添加新策略
2. 在`DecisionEngine`类中实现对应的`executeXxxStrategy`方法
3. 在`makeDecision`方法中添加策略分支

### 添加新的游戏状态

1. 在`GameState`类中添加新的状态变量和访问方法
2. 在决策引擎中考虑新状态的影响

### 自定义目标点评分

修改`DecisionEngine::calculateGoalScore`方法，调整权重和评分逻辑。

## 代码结构

```
goal_distribute/
├── include/
│   └── goal_distribute/
│       ├── game_state.hpp          # 游戏状态管理
│       ├── goal_manager.hpp        # 目标点管理
│       └── decision_engine.hpp     # 决策引擎
├── src/
│   ├── game_state.cpp
│   ├── goal_manager.cpp
│   ├── decision_engine.cpp
│   └── sentry_decision_node.cpp   # 主节点
├── launch/
│   └── sentry_decision.launch     # 启动文件
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 注意事项

1. **目标点配置**：确保在launch文件中配置了正确的目标点坐标
2. **游戏状态接口**：根据实际比赛系统的接口调整订阅的话题名称和消息格式
3. **参数调优**：根据实际比赛场景调整阈值和权重参数
4. **线程安全**：所有状态管理都使用了互斥锁保护，支持多线程访问

## 未来改进方向

- [ ] 添加历史决策记录和学习机制
- [ ] 实现更复杂的路径评分（考虑路径长度、危险程度等）
- [ ] 支持动态策略切换
- [ ] 添加决策可视化工具
- [ ] 支持多目标点序列规划

## 决策测试框架概览

为了替代裁判系统输入、快速验证完整决策逻辑，本项目提供了“前端 + 桥接 + ROS 服务”的闭环测试体系：

```
Web 前端 (decision_test_frontend.html)
    ↓ JSON POST /api/decision
HTTP 桥接 (decision_test_bridge.py)
    ↓ ROS Service /goal_distribute/decision_test
决策测试服务器 (decision_test_server.cpp)
    ↓ GameState + GoalManager + DecisionEngine
策略结果 (strategy/reason/confidence/goal_pose)
    ↓ 回传给前端
```

### 数据流细节

1. **前端输入**  
   - 文件：`scripts/decision_test_frontend.html`  
   - 录入自身血量/弹丸、友方/敌方建筑血量、敌人可见与坐标，点击“执行决策”发送 JSON。

2. **桥接服务器**  
   - 文件：`scripts/decision_test_bridge.py`  
   - 接收 `POST /api/decision` → 解析 JSON → 调用 ROS 服务 `/goal_distribute/decision_test`。  
   - 所有错误以 UTF-8 JSON 返回，端口自动重试。

3. **后端同步与决策**  
   - 文件：`src/decision_test_server.cpp`  
   - 将前端状态写入 `GameState`（`src/game_state.cpp`）、加载目标点（`src/goal_manager.cpp`），调用 `DecisionEngine::makeDecision()`。

4. **JSON 规则匹配**  
   - 文件：`src/decision_engine.cpp` + `config/decision_rules.json`  
   - `makeDecisionFromJSONRules()` 根据场地状态匹配规则，输出六类策略标签：  
     `SUPPLY_RETURN`、`SUPPLY_GUARD`、`ATTACK_ENEMY_BASE`、`ATTACK_ENEMY_OUTPOST`、`AGGRESSIVE_AIM`、`CONSERVATIVE_AIM`。

5. **结果回传前端**  
   - 桥接服务器将 ROS 响应打包为 JSON，前端展示策略、原因、置信度及目标点坐标。

### 启动步骤

```bash
cd ~/Code/Sentry25NAVI
catkin_make
source devel/setup.bash
roslaunch goal_distribute decision_test.launch
```

浏览器访问 `http://localhost:8080`（若端口被占用会自动切换并在日志中提示），即可输入场地状态并查看决策输出。
