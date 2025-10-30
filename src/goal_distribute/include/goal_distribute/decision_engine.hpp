#ifndef GOAL_DISTRIBUTE_DECISION_ENGINE_HPP
#define GOAL_DISTRIBUTE_DECISION_ENGINE_HPP

#include "goal_distribute/game_state.hpp"
#include "goal_distribute/goal_manager.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <string>

namespace goal_distribute {

/**
 * @brief 决策引擎
 * 基于游戏状态做出最优目标点决策
 */
class DecisionEngine {
public:
    // 决策策略枚举
    enum class Strategy {
        DEFENSIVE,      // 防御策略：保护关键建筑
        AGGRESSIVE,     // 攻击策略：主动出击
        BALANCED,       // 平衡策略：攻守兼备
        RETREAT,        // 撤退策略：回防基地
        REPOSITION      // 重新定位：寻找优势位置
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

    // ========== 辅助决策函数 ==========
    int selectBestGoalByPriority(const GameState& game_state, const GoalManager& goal_manager);
    int selectGoalNearBuilding(const GameState& game_state, const GoalManager& goal_manager, 
                              GameState::BuildingType building_type);
    int selectGoalAwayFromEnemy(const GameState& game_state, const GoalManager& goal_manager);
    double calculateGoalScore(int goal_id, const GameState& game_state, const GoalManager& goal_manager);

    Strategy default_strategy_;
    StrategyWeights weights_;
};

} // namespace goal_distribute

#endif // GOAL_DISTRIBUTE_DECISION_ENGINE_HPP
