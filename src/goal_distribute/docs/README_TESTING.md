# 决策系统前端测试指南

## 概述

决策系统提供了基于Web的前端测试界面，可以通过可视化方式设置游戏状态并查看决策结果。

## 系统架构

```
Web前端 (HTML) 
    ↓ HTTP请求
桥接服务器 (Python) 
    ↓ ROS服务调用
决策测试服务器 (C++) 
    ↓ 决策引擎
决策结果
```

## 快速开始

### 1. 编译项目

```bash
cd ~/Code/Sentry25NAVI
catkin_make
source devel/setup.bash
```

### 2. 启动测试服务器

```bash
roslaunch goal_distribute decision_test.launch
```

这将启动：
- 决策测试服务器（ROS服务：`/goal_distribute/decision_test`）
- 桥接服务器（HTTP服务器：`http://localhost:8080`）

### 3. 打开前端界面

在浏览器中打开：
```
http://localhost:8080
```

或者直接打开HTML文件：
```bash
firefox src/goal_distribute/scripts/decision_test_frontend.html
```

## 使用方法

### Web界面操作

1. **设置自身状态**
   - 自身血量：0-600
   - 自身弹丸：0-500

2. **设置建筑血量**
   - 友方基地血量：0-600
   - 友方前哨站血量：0-600
   - 敌方基地血量：0-600
   - 敌方前哨站血量：0-600

3. **设置敌人状态**
   - 勾选"敌人可见"复选框
   - 设置敌人坐标（X, Y, Z）

4. **执行决策**
   - 点击"执行决策"按钮
   - 查看决策结果

### 决策结果说明

- **目标点ID**: 选中的目标点编号（已与策略写死绑定，见下表）
- **策略**: 决策策略标签（SUPPLY_RETURN / ATTACK_ENEMY_BASE / ...）
- **决策原因**: 触发该决策的原因
- **置信度**: 决策的置信度（0-100%）
- **目标位置**: 目标点的坐标（X, Y, Z）

| 策略 | 固定目标ID |
|------|-----------|
| SUPPLY_RETURN | 0 |
| SUPPLY_GUARD | 1 |
| ATTACK_ENEMY_BASE | 2 |
| ATTACK_ENEMY_OUTPOST | 3 |
| AGGRESSIVE_AIM | 4 |
| CONSERVATIVE_AIM | 5 |
| DEFENSIVE | 6 |
| AGGRESSIVE | 7 |
| BALANCED | 8 |
| RETREAT | 9 |
| REPOSITION | 10 |

如需让结果中的 ID 对应真实地图位置，请在启动节点前调用 `GoalManager::applyGoalLayout()` 或在 launch 中配置相同编号的 `goal_num` 参数。

## 直接使用ROS服务

如果不使用Web界面，也可以直接调用ROS服务：

```bash
# 使用rosservice命令
rosservice call /goal_distribute/decision_test \
  "self_hp: 100
   self_ammo: 400
   ally_base_hp: 500
   ally_outpost_hp: 500
   enemy_base_hp: 600
   enemy_outpost_hp: 600
   enemy_visible: false
   enemy_x: 0.0
   enemy_y: 0.0
   enemy_z: 0.0"
```

## 测试场景示例

### 场景1: 低血量撤退
- 自身血量: 100 (< 30%)
- 其他状态: 正常
- 预期结果: RETREAT策略

### 场景2: 基地危险防御
- 基地血量: 200 (< 50%)
- 其他状态: 正常
- 预期结果: DEFENSIVE策略，目标点靠近基地

### 场景3: 敌人可见攻击
- 敌人可见: 是
- 自身血量: > 50%
- 自身弹丸: > 30%
- 预期结果: AGGRESSIVE策略

## 故障排除

### 问题1: 无法连接到服务器

**检查**:
1. 决策测试服务器是否运行：`rosnode list | grep decision_test`
2. 桥接服务器是否运行：检查端口8080是否被占用
3. 防火墙设置

### 问题2: 服务调用失败

**检查**:
1. 服务是否存在：`rosservice list | grep decision_test`
2. 查看服务信息：`rosservice info /goal_distribute/decision_test`
3. 查看节点日志：`rosnode info decision_test_server`

### 问题3: 决策结果不合理

**检查**:
1. JSON规则文件是否正确加载
2. 目标点配置是否正确
3. 查看DEBUG日志：`rosrun goal_distribute decision_test_server _log_level:=DEBUG`

## 文件说明

- `srv/DecisionTest.srv`: ROS服务定义
- `src/decision_test_server.cpp`: 决策测试服务器实现
- `scripts/decision_test_bridge.py`: HTTP到ROS的桥接服务
- `scripts/decision_test_frontend.html`: Web前端界面
- `launch/decision_test.launch`: 启动文件

## 自定义配置

### 修改端口

编辑 `launch/decision_test.launch`:
```xml
<param name="port" value="8080" />
```

### 修改JSON规则

编辑 `config/decision_rules.json`，然后重启服务器。

### 修改目标点配置

在launch文件中加载目标点YAML文件，或通过ROS参数服务器设置。

