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
 * @brief 目标选择配置（简化版，只支持固定ID）
 */
struct GoalSelectionConfig {
    std::string type = "fixed_id";  // 现在只支持 "fixed_id"，其他类型已移除
    int goal_id = -1;  // 直接指定目标点ID（-1表示使用策略的固定映射）
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

