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

将JSON文件放在 `config/decision_rules.json`，节点会自动加载。

### 方法2: 通过ROS参数指定

在launch文件中指定：

```xml
<node name="sentry_decision_node" pkg="goal_distribute" type="sentry_decision_node">
  <param name="json_rules_file" value="$(find goal_distribute)/config/decision_rules.json" />
</node>
```

### 方法3: 在代码中加载

```cpp
decision_engine.loadRulesFromJSON("/path/to/rules.json");
decision_engine.setUseJSONRules(true);
```

## 切换回传统方法

如果不想使用JSON规则，可以：

1. 不提供JSON配置文件（节点会自动使用传统方法）
2. 在代码中设置：
```cpp
decision_engine.setUseJSONRules(false);
```

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

