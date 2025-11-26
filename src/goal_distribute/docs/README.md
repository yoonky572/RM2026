# goal_distribute 模块说明

## 1. 模块定位

`goal_distribute` 负责 RoboMaster 哨兵的“高层决策 + 目标点下发”。模块以 ROS 节点形式运行，实时接收裁判系统及自有传感信息，输出最优目标点到导航系统。  
**目前决策完全由 JSON 规则驱动**（无传统 if-else 回退），策略标签与目标点编号一一对应。其核心优势：

- **可扩展**：支持多策略权重、多规则文件、目标点动态管理。
- **可观测**：提供测试前端 + 桥接 + ROS 服务的闭环验证流程。
- **可维护**：代码层级清晰，GameState / GoalManager / DecisionEngine 各司其职。

---

## 2. 目录结构

```
goal_distribute/
├── include/goal_distribute/
│   ├── game_state.hpp          # 游戏状态封装
│   ├── goal_manager.hpp        # 目标点及优先级管理
│   ├── decision_engine.hpp     # 策略与评分
│   └── decision_test_server.hpp# 测试服务接口
├── src/
│   ├── game_state.cpp
│   ├── goal_manager.cpp
│   ├── decision_engine.cpp
│   ├── sentry_decision_node.cpp
│   └── decision_test_server.cpp
├── scripts/
│   ├── decision_test_bridge.py
│   └── decision_test_frontend.html
├── config/
│   ├── goal_points.yaml
│   └── decision_rules.json
├── launch/
│   ├── sentry_decision.launch
│   └── decision_test.launch
├── srv/
│   └── DecisionTest.srv
├── docs/
│   └── readme.md（更多设计细节）
├── CMakeLists.txt
├── package.xml
└── README.md（本文件）
```

---

## 3. 架构概览

| 组件 | 职责 |
|------|------|
| `GameState` | 订阅/记录血量、弹丸、建筑、敌人等原始信息，提供高级状态查询（血量是否危险等） |
| `GoalManager` | 从参数加载或动态维护目标点；支持优先级、启用状态、目标标签等属性 |
| `DecisionEngine` | 根据 GameState + GoalManager + JSON 规则计算策略标签（SUPPLY_RETURN 等）并输出固定目标点 ID |
| `SentryDecisionNode` | ROS 主节点：整合上述组件，定时触发决策、发布导航目标 |
| `Decision Test Stack` | `decision_test_frontend` + `decision_test_bridge` + `decision_test_server`，用于离线验证策略 |

### 模块数据流（简述）

1. **输入**：`/game_state/self_hp`、`/game_state/self_ammo`、`/game_state/building_hp`、`/game_state/enemy` 等话题。
2. **处理**：
   - GameState 更新内部状态。
   - GoalManager 提供可选目标集合。
   - DecisionEngine 运行策略或 JSON 规则匹配，计算评分。
3. **输出**：`/move_base_simple/goal`（导航终点）、`/integer_topic`（目标点 ID）、`/sentry_strategy`（理由）。

---

## 4. 决策循环

1. **状态同步**：GameState 接收消息后持久化（加锁保护）。
2. **策略选择**：按优先级遍历 JSON 规则，命中后返回策略标签（SUPPLY_RETURN / ATTACK_ENEMY_BASE 等）。若未命中，使用保守兜底逻辑。
3. **目标绑定**：
   - 解析 `goal_selection`；若未选出目标，则使用内置“策略→固定ID”映射。
   - `GoalManager` 负责根据 ID 返回 Pose。
4. **结果发布**：输出 PoseStamped + 固定目标 ID + 策略说明，供导航及上层系统使用。

### Baseline 策略标签

| 策略 | 默认编号 | 典型触发条件 | 行为特征 |
|------|------|--------------|----------|
| SUPPLY_RETURN | 0 | 自身血量/弹量过低 | 立即返回补给点 |
| SUPPLY_GUARD | 1 | 友方建筑受压 | 回城驻守并保持火力 |
| ATTACK_ENEMY_BASE | 2 | 敌方基地虚弱且自身状态好 | 前往敌方基地攻击 |
| ATTACK_ENEMY_OUTPOST | 3 | 敌前哨血量低 | 压制前哨站 |
| AGGRESSIVE_AIM | 4 | 敌人可见或我方优势 | 选择前沿火力点 |
| CONSERVATIVE_AIM | 5 | 我方建筑危险但自身尚可 | 留守关键建筑周边 |

历史上保留的 DEFENSIVE / AGGRESSIVE / BALANCED / RETREAT / REPOSITION 仍在枚举中，并占用编号 6~10（详见下文固定映射），可用于兜底策略。

---

## 5. 固定策略编号与目标点布局

### 5.1 内置编号（写死顺序）

| 编号 | 策略 | 含义 |
| --- | --- | --- |
| 0 | SUPPLY_RETURN | 回城补给：血/弹极低时立刻返回补给点 |
| 1 | SUPPLY_GUARD | 回城守护：友方建筑受压，返回基地防守 |
| 2 | ATTACK_ENEMY_BASE | 攻击敌基地：敌方基地虚弱，主攻其核心 |
| 3 | ATTACK_ENEMY_OUTPOST | 攻击敌前哨：针对敌前哨站的压制行动 |
| 4 | AGGRESSIVE_AIM | 激进占位：敌可见或我方优势时前压找火力点 |
| 5 | CONSERVATIVE_AIM | 保守占位：守在己方关键建筑周围压枪 |
| 6 | DEFENSIVE | 防御巡逻：优先保卫基地/前哨的巡逻策略 |
| 7 | AGGRESSIVE | 进攻巡逻：持续向前沿推进寻找攻击机会 |
| 8 | BALANCED | 平衡巡逻：根据综合评分寻找通用点位 |
| 9 | RETREAT | 撤退：主动远离威胁，寻找安全位置 |
| 10 | REPOSITION | 机动换位：重新部署到评分更高的点 |

规则文件只要返回对应策略，决策引擎就会自动填入该 ID，不需要额外判断。

### 5.2 目标点落地方式

- **一次性布局**：`GoalManager::applyGoalLayout()` 支持把上述 ID 与实际地图 Pose 批量绑定，适合在节点初始化或地图切换时调用。
- **ROS 参数**：仍然可以在 launch 中通过 `goal_num` + `red_goalX` 方式加载，保持向后兼容。

目前在 `launch/decision_test.launch` 中给出了默认示例坐标，方便立刻测试。每一行参数包含 `[x, y, z, qx, qy, qz, qw]`，例如：

| ID | 策略 | Launch 默认坐标 (x, y) | 说明 |
|----|------|------------------------|------|
| 0 | SUPPLY_RETURN | (0.5, -2.5) | 回城补给点（己方战区后方） |
| 1 | SUPPLY_GUARD | (0.5, -1.5) | 基地守卫点 |
| 2 | ATTACK_ENEMY_BASE | (6.0, 2.0) | 敌基地攻击路径终点 |
| 3 | ATTACK_ENEMY_OUTPOST | (5.0, 3.5) | 敌前哨压制点 |
| 4 | AGGRESSIVE_AIM | (3.5, 1.0) | 前沿火力位 |
| 5 | CONSERVATIVE_AIM | (1.5, -0.5) | 基地附近保守位 |
| 6 | DEFENSIVE | (0.8, -0.8) | 防御巡逻点 |
| 7 | AGGRESSIVE | (2.8, 1.8) | 进攻巡逻点 |
| 8 | BALANCED | (2.0, 0.5) | 平衡巡逻点 |
| 9 | RETREAT | (0.0, -3.0) | 撤退缓冲区 |
| 10 | REPOSITION | (1.5, 1.5) | 机动换位点 |

蓝方（`blue_goalX`）在同一文件中使用镜像坐标，便于切换 `team_color`。实际部署时只需替换这些数值即可。

示例：

```cpp
std::vector<goal_distribute::GoalManager::GoalLayoutEntry> layout = {
    {0, makePose(0.8, -1.2, 0.0), "supply_return_home", 10},
    {1, makePose(1.0, -0.8, 0.0), "guard_base", 9},
    // ... 补齐 2~10
};
goal_manager.applyGoalLayout(layout);
```

地图设计负责人只需要维护这一份列表，即可保证“策略=主键=目标点”。

---

## 6. ROS 接口

### 订阅
- `/game_state/self_hp` (std_msgs/Int32)
- `/game_state/self_ammo` (std_msgs/Int32)
- `/game_state/building_hp` (std_msgs/Int32 或自定义)
- `/game_state/enemy` (geometry_msgs/PoseStamped)

### 发布
- `/move_base_simple/goal` (geometry_msgs/PoseStamped)
- `/integer_topic` (std_msgs/Int32，目标点 ID)
- `/sentry_strategy` (std_msgs/String，策略与原因)

### 参数示例（launch）
- `decision_rate`：决策频率（默认 10 Hz）
- `default_strategy`：0~4（DEFENSIVE/AGGRESSIVE/BALANCED/RETREAT/REPOSITION）
- `team_color`：`"red"` or `"blue"`
- `initial_hp`：默认 600
- `initial_ammo`：默认 500

---

## 7. 决策测试闭环

> 目的：在无裁判系统输入时模拟战场状态，快速验证决策输出。

| 阶段 | 描述 |
|------|------|
| 前端 (`decision_test_frontend.html`) | 浏览器输入血量、弹丸、建筑情况、敌人坐标，点击“执行决策” |
| 规则构建前端 (同上) | 「规则构建」页签可拼装/删除 JSON 规则并一键写入 `config/decision_rules.json`，省去手写 JSON |
| 桥接 (`decision_test_bridge.py`) | 提供 HTTP API，转发 JSON -> ROS Service `/goal_distribute/decision_test` |
| 服务 (`decision_test_server.cpp`) | 调用 GameState/GoalManager/DecisionEngine，返回策略、原因、目标点 |
| 规则匹配 | `decision_rules.json` + `makeDecisionFromJSONRules()`，输出六类标签（SUPPLY_RETURN 等） |

### 启动流程
```bash
cd ~/Code/Sentry25NAVI
catkin_make
source devel/setup.bash
roslaunch goal_distribute decision_test.launch
# 浏览器访问 http://localhost:8080
# 规则更新后需要重启/重新加载测试服务器才能生效
```

---

## 8. 开发指南

### 8.1 新策略接入
1. 在 `DecisionEngine::Strategy` 枚举中新增标签（若需要新的编号）。
2. 在 JSON 规则中添加 `strategy` 字段引用该标签，并编写触发条件。
3. 为新的编号分配目标点 Pose（`applyGoalLayout` 或 ROS 参数）。

**无需再实现 `executeXxxStrategy()`/`switch` 逻辑**，所有决策流程都走 JSON 规则。

### 8.2 扩展 GameState
1. 添加成员变量及读写接口。
2. 在消息回调中更新数据。
3. 在 `DecisionEngine` 中读取并融入评分。

### 8.3 自定义评分
修改 `DecisionEngine::calculateGoalScore()` 或在 JSON 中调整 `weights`/`goal_selection`，即可影响 `best_score` 类型的目标选择。

---

## 9. 注意事项 & TODO

- 请确保 launch 中的目标点坐标与实际地图一致。
- 根据比赛接口调整话题名称与消息格式。
- 所有共享数据均使用 mutex 保护，保证多线程安全。

未来改进：
- [ ] 决策历史与学习机制
- [ ] 更复杂的路径/风险评分
- [ ] 策略可视化与调试工具
- [ ] 多目标序列规划

---

该 README 根据当前架构重新整理，方便快速了解 `goal_distribute` 模块的功能、baseline 行为及扩展方法。更多设计细节可参考 `docs/readme.md`。
