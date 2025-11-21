#ifndef GOAL_DISTRIBUTE_DECISION_RULE_CONFIG_HPP
#define GOAL_DISTRIBUTE_DECISION_RULE_CONFIG_HPP

#include <string>
#include <map>
#include <memory>

namespace goal_distribute {

/**
 * @brief 条件配置结构
 */
struct ConditionValue {
    std::string operator_;  // "lt", "lte", "gt", "gte", "eq", "ne"
    double value;
};

/**
 * @brief 条件配置
 */
struct ConditionConfig {
    std::map<std::string, ConditionValue> numeric_conditions;  // 数值条件
    std::map<std::string, bool> bool_conditions;  // 布尔条件
};

/**
 * @brief 目标选择配置
 */
struct GoalSelectionConfig {
    std::string type;  // "near_building", "near_enemy", "away_from_enemy", "best_score", "by_priority"
    std::string building_type;  // "BASE", "SENTRY_OUTPOST" (当type为near_building时)
    double ideal_distance;  // 理想距离 (当type为near_enemy时)
    int min_priority;  // 最小优先级 (当type为by_priority时)
};

/**
 * @brief 动作配置
 */
struct ActionConfig {
    std::string strategy;  // "DEFENSIVE", "AGGRESSIVE", "BALANCED", "RETREAT", "REPOSITION"
    GoalSelectionConfig goal_selection;
    double confidence;
    std::string reason;
};

/**
 * @brief 规则配置
 */
struct RuleConfig {
    std::string name;
    int priority;
    ConditionConfig conditions;
    ActionConfig action;
};

} // namespace goal_distribute

#endif // GOAL_DISTRIBUTE_DECISION_RULE_CONFIG_HPP

