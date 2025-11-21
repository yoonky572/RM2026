#ifndef GOAL_DISTRIBUTE_DECISION_ENGINE_HPP
#define GOAL_DISTRIBUTE_DECISION_ENGINE_HPP

#include "goal_distribute/game_state.hpp"
#include "goal_distribute/goal_manager.hpp"
#include "goal_distribute/decision_rule_config.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <string>
#include <vector>
#include <memory>
#include <fstream>

namespace goal_distribute {

/**
 * @brief 决策引擎
 * 基于游戏状态做出最优目标点决策
 */
class DecisionEngine {
public:
    // 决策策略枚举
    enum class Strategy {
        // 传统策略（作为后备）
        DEFENSIVE,
        AGGRESSIVE,
        BALANCED,
        RETREAT,
        REPOSITION,
        // 前端测试所需的场景状态标签
        SUPPLY_RETURN,
        SUPPLY_GUARD,
        ATTACK_ENEMY_BASE,
        ATTACK_ENEMY_OUTPOST,
        AGGRESSIVE_AIM,
        CONSERVATIVE_AIM,
        UNKNOWN
    };

    // 决策结果
    struct Decision {
        int goal_id;
        geometry_msgs::PoseStamped goal_pose;
        Strategy strategy;
        std::string reason;  // 决策原因
        double confidence;  // 决策置信度 [0.0, 1.0]
    };

    DecisionEngine();
    ~DecisionEngine() = default;

    // ========== 主要决策接口 ==========
    Decision makeDecision(const GameState& game_state, const GoalManager& goal_manager);

    // ========== JSON规则配置 ==========
    /**
     * @brief 从JSON文件加载决策规则
     * @param json_file_path JSON配置文件路径
     * @return 是否加载成功
     */
    bool loadRulesFromJSON(const std::string& json_file_path);
    
    /**
     * @brief 启用/禁用JSON规则模式
     * @param enable true表示使用JSON规则，false表示使用传统if-else方法
     */
    void setUseJSONRules(bool enable);
    
    /**
     * @brief 检查是否使用JSON规则
     */
    bool isUsingJSONRules() const { return use_json_rules_; }

    // ========== 策略设置 ==========
    void setDefaultStrategy(Strategy strategy);
    Strategy getDefaultStrategy() const;

    // ========== 权重配置 ==========
    struct StrategyWeights {
        double hp_weight = 0.3;
        double ammo_weight = 0.2;
        double building_weight = 0.3;
        double enemy_weight = 0.2;
    };
    void setStrategyWeights(const StrategyWeights& weights);

private:
    // ========== 决策策略实现 ==========
    Decision executeDefensiveStrategy(const GameState& game_state, const GoalManager& goal_manager);
    Decision executeAggressiveStrategy(const GameState& game_state, const GoalManager& goal_manager);
    Decision executeBalancedStrategy(const GameState& game_state, const GoalManager& goal_manager);
    Decision executeRetreatStrategy(const GameState& game_state, const GoalManager& goal_manager);
    Decision executeRepositionStrategy(const GameState& game_state, const GoalManager& goal_manager);

    // ========== JSON规则相关 ==========
    /**
     * @brief 基于JSON规则进行决策
     */
    Decision makeDecisionFromJSONRules(const GameState& game_state, const GoalManager& goal_manager);
    
    /**
     * @brief 检查条件是否满足
     */
    bool checkConditions(const ConditionConfig& conditions, const GameState& game_state);
    
    /**
     * @brief 执行动作
     */
    Decision executeAction(const ActionConfig& action, const GameState& game_state, 
                          const GoalManager& goal_manager);

    // ========== 辅助决策函数 ==========
    int selectBestGoalByPriority(const GameState& game_state, const GoalManager& goal_manager);
    int selectGoalNearBuilding(const GameState& game_state, const GoalManager& goal_manager, 
                              GameState::BuildingType building_type);
    int selectGoalAwayFromEnemy(const GameState& game_state, const GoalManager& goal_manager);
    int selectGoalNearEnemy(const GameState& game_state, const GoalManager& goal_manager, 
                           double ideal_distance);
    double calculateGoalScore(int goal_id, const GameState& game_state, const GoalManager& goal_manager);

    Strategy default_strategy_;
    StrategyWeights weights_;
    bool use_json_rules_;
    std::vector<std::unique_ptr<RuleConfig>> json_rules_;  // JSON规则列表（按优先级排序）
};

} // namespace goal_distribute

#endif // GOAL_DISTRIBUTE_DECISION_ENGINE_HPP
