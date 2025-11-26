# JSON决策规则使用指南

## 概述

决策引擎现在支持基于JSON配置文件的规则驱动决策，替代了传统的if-else嵌套方法。这种方式更加灵活，易于修改和维护。

## 配置文件格式

JSON配置文件位于 `config/decision_rules.json`，包含以下结构：

```json
{
  "version": "1.0",
  "description": "决策规则描述",
  "rules": [
    {
      "name": "规则名称",
      "priority": 优先级数字（越大越优先）,
      "conditions": {
        "条件键": {
          "operator": "操作符",
          "value": 阈值
        }
      },
      "action": {
        "strategy": "策略名称",
        "goal_selection": {
          "type": "目标选择类型",
          "参数": "参数值"
        },
        "confidence": 置信度,
        "reason": "决策原因"
      }
    }
  ],
  "weights": {
    "hp_weight": 0.3,
    "ammo_weight": 0.2,
    "building_weight": 0.3,
    "enemy_weight": 0.2
  }
}
```

## 条件配置

### 数值条件

支持的键：
- `self_hp_percent`: 自身血量百分比 [0.0, 1.0]
- `self_ammo_percent`: 自身弹丸百分比 [0.0, 1.0]
- `base_hp_percent`: 基地血量百分比 [0.0, 1.0]
- `outpost_hp_percent`: 前哨站血量百分比 [0.0, 1.0]
- `enemy_base_hp_percent`: 敌方基地血量百分比 [0.0, 1.0]
- `enemy_outpost_hp_percent`: 敌方前哨站血量百分比 [0.0, 1.0]

支持的操作符：
- `lt`: 小于 (<)
- `lte`: 小于等于 (<=)
- `gt`: 大于 (>)
- `gte`: 大于等于 (>=)
- `eq`: 等于 (==)
- `ne`: 不等于 (!=)

示例：
```json
"conditions": {
  "self_hp_percent": {
    "operator": "lt",
    "value": 0.3
  }
}
```

### 布尔条件

支持的键：
- `enemy_visible`: 敌人是否可见 (true/false)

示例：
```json
"conditions": {
  "enemy_visible": true
}
```

### 组合条件

多个条件之间是**AND**关系（所有条件都必须满足）：

```json
"conditions": {
  "self_hp_percent": {
    "operator": "gte",
    "value": 0.5
  },
  "self_ammo_percent": {
    "operator": "gte",
    "value": 0.3
  },
  "enemy_visible": true
}
```

## 动作配置

### 策略类型（主决策状态）

主规则只负责识别场地状态，并返回以下六种“策略标签”。具体执行逻辑由上层业务根据标签自行处理：

- `SUPPLY_RETURN`：回城补给
- `SUPPLY_GUARD`：回城自瞄守护
- `ATTACK_ENEMY_BASE`：进攻敌方基地
- `ATTACK_ENEMY_OUTPOST`：进攻敌方前哨站
- `AGGRESSIVE_AIM`：激进位置自瞄
- `CONSERVATIVE_AIM`：保守位置自瞄

### 目标选择类型

#### 1. `near_building` - 靠近建筑

选择靠近指定建筑的目标点。

参数：
- `building_type`: `"BASE"` 或 `"SENTRY_OUTPOST"`

示例：
```json
"goal_selection": {
  "type": "near_building",
  "building_type": "BASE"
}
```

#### 2. `near_enemy` - 靠近敌人

选择靠近敌人的目标点。

参数：
- `ideal_distance`: 理想距离（米）

示例：
```json
"goal_selection": {
  "type": "near_enemy",
  "ideal_distance": 3.0
}
```

#### 3. `away_from_enemy` - 远离敌人

选择远离敌人的目标点。

示例：
```json
"goal_selection": {
  "type": "away_from_enemy"
}
```

#### 4. `best_score` - 最佳评分

根据综合评分选择最佳目标点。

示例：
```json
"goal_selection": {
  "type": "best_score"
}
```

#### 5. `by_priority` - 按优先级

选择指定优先级以上的目标点。

参数：
- `min_priority`: 最小优先级

示例：
```json
"goal_selection": {
  "type": "by_priority",
  "min_priority": 5
}
```

#### 6. `fixed_id` / 显式 `goal_id`

当需要让**某个策略始终对应同一个目标点ID**时，可以直接在 `goal_selection` 中提供 `goal_id`。  
填写方式有两种：

- 保持原有 `type`（例如 `near_building`），同时增加 `goal_id` 字段。若ID有效，将优先生效；
- 设置 `type` 为 `"fixed_id"` 并配合 `goal_id`。

示例：

```json
"goal_selection": {
  "type": "fixed_id",
  "goal_id": 4
}
```

一旦 `goal_id` 被解析成功，决策引擎会直接返回对应的目标点，不再走其它选择逻辑。这可以保证“一个策略对应一个固定点位”。

### 策略固定编号（内置顺序）

即使 JSON 中没有显式提供 `goal_id`，引擎也会按“数据库主键”思路，为每个策略写死一个目标点编号，顺序如下：

| 编号 | 策略标签 | 策略含义 |
| --- | --- | --- |
| 0 | SUPPLY_RETURN | 补给：自身血/弹不足，立即回城 |
| 1 | SUPPLY_GUARD | 补给守护：回基地守住关键建筑 |
| 2 | ATTACK_ENEMY_BASE | 打敌基地：敌基地虚弱时推进 |
| 3 | ATTACK_ENEMY_OUTPOST | 打敌前哨：压制敌前哨站 |
| 4 | AGGRESSIVE_AIM | 激进占位：前线高压火力点 |
| 5 | CONSERVATIVE_AIM | 保守占位：基地/前哨附近防守点 |
| 6 | DEFENSIVE | 防御巡逻：优先守护己方建筑 |
| 7 | AGGRESSIVE | 进攻巡逻：向敌方向推进 |
| 8 | BALANCED | 平衡巡逻：综合权重的默认点位 |
| 9 | RETREAT | 撤退：远离敌人寻找安全位 |
| 10 | REPOSITION | 重新部署：换到更优位置 |

因此：  
- 如果规则返回 `strategy="SUPPLY_RETURN"` 且未指定 `goal_id`，最终仍会落在编号 0；  
- 设计同学只需要把上述 ID 与实际地图坐标一一对应即可。

### 目标点布局接口

`GoalManager` 新增 `applyGoalLayout()`，可以在任意初始化阶段一次性写入“编号 → 具体 Pose”。示例：

```cpp
#include "goal_distribute/goal_manager.hpp"

std::vector<goal_distribute::GoalManager::GoalLayoutEntry> layout = {
    {0, makePose(1.0, 2.0, 0.0), "supply_return_home", 10},
    {1, makePose(1.5, 2.5, 0.0), "supply_guard_base", 9},
    {2, makePose(5.0, 3.2, 0.0), "attack_enemy_base", 8},
    // ... 继续补完 3~10
};

goal_manager.applyGoalLayout(layout);
```

这样既能保证“策略=主键”，又能让地图坐标保持在配置/代码中可控的位置；若后续需要换图，只要重新构建 `layout` 并调用一次接口即可。

## 完整示例

```json
{
  "name": "紧急撤退",
  "priority": 100,
  "conditions": {
    "self_hp_percent": {
      "operator": "lt",
      "value": 0.3
    }
  },
  "action": {
    "strategy": "RETREAT",
    "goal_selection": {
      "type": "near_building",
      "building_type": "BASE"
    },
    "confidence": 0.95,
    "reason": "紧急撤退: 自身血量过低"
  }
}
```

## 规则优先级

规则按 `priority` 字段从高到低排序。决策引擎会按顺序检查规则，执行**第一个**满足所有条件的规则。

**重要**：确保高优先级规则的条件更加严格，避免低优先级规则被忽略。

## 使用方法

### 方法1: 使用默认配置文件

将JSON文件放在 `config/decision_rules.json`，节点会自动加载。若需要拆分为多个文件，可将 `json_rules_file` 参数指向一个目录，目录下所有 `.json` 文件都会被合并加载。

### 方法2: 通过ROS参数指定

在launch文件中指定：

```xml
<node name="sentry_decision_node" pkg="goal_distribute" type="sentry_decision_node">
  <param name="json_rules_file" value="$(find goal_distribute)/config/decision_rules.json" />
</node>
```

### 方法3: 在代码中加载

```cpp
decision_engine.loadRulesFromJSON("/path/to/rules_or_directory");
decision_engine.setUseJSONRules(true);
```

## 可视化规则构建前端

- 启动 `roslaunch goal_distribute decision_test.launch` 后，访问 `http://localhost:8080`。
- 在页面顶部切换到 **「规则构建」** 页签，可填写条件、策略、目标选择等信息：
  - 百分比阈值以 0~100 的形式输入，前端会自动转换为 0~1。
  - 目标选择支持 near_building / near_enemy / best_score / by_priority / fixed_id 等，并可附带 `goal_id` 覆盖。
  - “生成规则 JSON” 可预览最终结构，确认无误后点击“写入配置”即会更新 `config/decision_rules.json`。
- 规则列表支持一键删除（按钮带确认提示），删除后同样需要重启或触发规则重新加载。
- 由于 `decision_test_bridge.py` 直接写入 JSON 文件，**更新后请重启决策测试节点或在正式节点中重新加载规则**，以确保新规则被引擎读取。


## 拆分策略文件

可以将不同策略的规则拆分为独立JSON文件，例如：

- `config/rules/defensive.json`
- `config/rules/aggressive.json`
- `config/rules/balanced.json`
- `config/rules/reposition.json`

将 `json_rules_file` 参数指向 `config/rules/` 目录后，决策引擎会自动读取上述文件并按优先级合并。这样就无需在代码中维护 `if-else` 或 `switch`，每个策略只需在对应的JSON文件内描述条件和动作即可。

## 调试技巧

1. **查看匹配的规则**：启用ROS调试日志
   ```bash
   rosrun goal_distribute sentry_decision_node _log_level:=DEBUG
   ```

2. **验证JSON格式**：使用在线JSON验证工具或命令行：
   ```bash
   python3 -m json.tool config/decision_rules.json
   ```

3. **测试单个规则**：临时提高某个规则的优先级，观察是否被触发

## 注意事项

1. JSON文件必须是有效的JSON格式
2. 规则优先级数字越大，优先级越高
3. 如果所有规则都不匹配，会使用默认的fallback策略
4. 条件检查是顺序的，第一个满足条件的规则会被执行
5. 修改JSON文件后需要重启节点才能生效

## 权重配置

可以在JSON文件中配置评分权重：

```json
"weights": {
  "hp_weight": 0.3,
  "ammo_weight": 0.2,
  "building_weight": 0.3,
  "enemy_weight": 0.2
}
```

这些权重用于 `best_score` 类型的目标选择。

