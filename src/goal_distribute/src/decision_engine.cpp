#include "goal_distribute/decision_engine.hpp"
#include <algorithm>
#include <cmath>
#include <ros/ros.h>

namespace goal_distribute {

DecisionEngine::DecisionEngine()
    : default_strategy_(Strategy::BALANCED)
{
    weights_.hp_weight = 0.3;
    weights_.ammo_weight = 0.2;
    weights_.building_weight = 0.3;
    weights_.enemy_weight = 0.2;
}

DecisionEngine::Decision DecisionEngine::makeDecision(
    const GameState& game_state, const GoalManager& goal_manager) {
    
    Decision decision;
    decision.confidence = 0.0;
    decision.goal_id = -1;

    // 首先判断是否需要撤退
    if (game_state.needRetreat()) {
        decision = executeRetreatStrategy(game_state, goal_manager);
        decision.reason = "Retreat: Low HP or buildings in danger";
        return decision;
    }

    // 根据默认策略执行决策
    switch (default_strategy_) {
        case Strategy::DEFENSIVE:
            decision = executeDefensiveStrategy(game_state, goal_manager);
            break;
        case Strategy::AGGRESSIVE:
            decision = executeAggressiveStrategy(game_state, goal_manager);
            break;
        case Strategy::BALANCED:
            decision = executeBalancedStrategy(game_state, goal_manager);
            break;
        case Strategy::REPOSITION:
            decision = executeRepositionStrategy(game_state, goal_manager);
            break;
        default:
            decision = executeBalancedStrategy(game_state, goal_manager);
            break;
    }

    // 如果决策失败，使用优先级选择
    if (decision.goal_id < 0) {
        decision.goal_id = selectBestGoalByPriority(game_state, goal_manager);
        if (decision.goal_id >= 0) {
            goal_manager.getGoalPoint(decision.goal_id, decision.goal_pose);
            decision.confidence = 0.5;
            decision.reason = "Fallback: Selected by priority";
        }
    }

    return decision;
}

DecisionEngine::Decision DecisionEngine::executeDefensiveStrategy(
    const GameState& game_state, const GoalManager& goal_manager) {
    
    Decision decision;
    decision.strategy = Strategy::DEFENSIVE;
    
    // 防御策略：优先保护基地和前哨站
    if (game_state.isBaseInDanger()) {
        decision.goal_id = selectGoalNearBuilding(game_state, goal_manager, 
                                                   GameState::BuildingType::BASE);
        if (decision.goal_id >= 0) {
            goal_manager.getGoalPoint(decision.goal_id, decision.goal_pose);
            decision.confidence = 0.9;
            decision.reason = "Defensive: Protecting base";
            return decision;
        }
    }
    
    if (game_state.isOutpostInDanger()) {
        decision.goal_id = selectGoalNearBuilding(game_state, goal_manager, 
                                                  GameState::BuildingType::SENTRY_OUTPOST);
        if (decision.goal_id >= 0) {
            goal_manager.getGoalPoint(decision.goal_id, decision.goal_pose);
            decision.confidence = 0.85;
            decision.reason = "Defensive: Protecting outpost";
            return decision;
        }
    }
    
    // 如果建筑安全，选择防御性目标点
    auto goals = goal_manager.getGoalsByPriority(5);  // 高优先级目标
    if (!goals.empty()) {
        decision.goal_id = goals[0];
        goal_manager.getGoalPoint(decision.goal_id, decision.goal_pose);
        decision.confidence = 0.7;
        decision.reason = "Defensive: High priority defensive position";
    }
    
    return decision;
}

DecisionEngine::Decision DecisionEngine::executeAggressiveStrategy(
    const GameState& game_state, const GoalManager& goal_manager) {
    
    Decision decision;
    decision.strategy = Strategy::AGGRESSIVE;
    
    // 攻击策略：如果敌人可见，选择接近敌人的位置
    if (game_state.isEnemyVisible()) {
        // 根据敌人位置和自身状态选择最佳攻击位置
        // 这里简化处理，实际应该考虑敌人位置和掩护位置
        double enemy_x, enemy_y, enemy_z;
        game_state.getEnemyPosition(enemy_x, enemy_y, enemy_z);
        
        auto goals = goal_manager.getAvailableGoals();
        double max_score = -1.0;
        int best_goal_id = -1;
        
        for (int goal_id : goals) {
            geometry_msgs::PoseStamped pose;
            if (goal_manager.getGoalPoint(goal_id, pose)) {
                double score = calculateGoalScore(goal_id, game_state, goal_manager);
                // 优先选择距离敌人适中且评估分数高的位置
                double dx = pose.pose.position.x - enemy_x;
                double dy = pose.pose.position.y - enemy_y;
                double distance = std::sqrt(dx*dx + dy*dy);
                // 理想攻击距离（根据实际情况调整）
                double ideal_distance = 3.0;
                double distance_score = 1.0 / (1.0 + std::abs(distance - ideal_distance));
                
                score = score * 0.6 + distance_score * 0.4;
                
                if (score > max_score) {
                    max_score = score;
                    best_goal_id = goal_id;
                }
            }
        }
        
        if (best_goal_id >= 0) {
            decision.goal_id = best_goal_id;
            goal_manager.getGoalPoint(decision.goal_id, decision.goal_pose);
            decision.confidence = 0.8;
            decision.reason = "Aggressive: Engaging enemy";
        }
    }
    
    // 如果敌人不可见，选择前沿位置
    if (decision.goal_id < 0) {
        auto goals = goal_manager.getGoalsByPriority(3);
        if (!goals.empty()) {
            decision.goal_id = goals[0];
            goal_manager.getGoalPoint(decision.goal_id, decision.goal_pose);
            decision.confidence = 0.6;
            decision.reason = "Aggressive: Forward position";
        }
    }
    
    return decision;
}

DecisionEngine::Decision DecisionEngine::executeBalancedStrategy(
    const GameState& game_state, const GoalManager& goal_manager) {
    
    Decision decision;
    decision.strategy = Strategy::BALANCED;
    
    // 平衡策略：综合考虑所有因素
    auto goals = goal_manager.getAvailableGoals();
    double max_score = -1.0;
    int best_goal_id = -1;
    
    for (int goal_id : goals) {
        double score = calculateGoalScore(goal_id, game_state, goal_manager);
        if (score > max_score) {
            max_score = score;
            best_goal_id = goal_id;
        }
    }
    
    if (best_goal_id >= 0) {
        decision.goal_id = best_goal_id;
        goal_manager.getGoalPoint(decision.goal_id, decision.goal_pose);
        decision.confidence = std::min(max_score, 1.0);
        decision.reason = "Balanced: Optimal position based on all factors";
    }
    
    return decision;
}

DecisionEngine::Decision DecisionEngine::executeRetreatStrategy(
    const GameState& game_state, const GoalManager& goal_manager) {
    
    Decision decision;
    decision.strategy = Strategy::RETREAT;
    
    // 撤退策略：选择安全的、靠近基地的位置
    decision.goal_id = selectGoalNearBuilding(game_state, goal_manager, 
                                             GameState::BuildingType::BASE);
    
    if (decision.goal_id >= 0) {
        goal_manager.getGoalPoint(decision.goal_id, decision.goal_pose);
        decision.confidence = 0.95;
        decision.reason = "Retreat: Returning to base for safety";
    } else {
        // 如果没有找到基地附近的目标，选择远离敌人的位置
        decision.goal_id = selectGoalAwayFromEnemy(game_state, goal_manager);
        if (decision.goal_id >= 0) {
            goal_manager.getGoalPoint(decision.goal_id, decision.goal_pose);
            decision.confidence = 0.85;
            decision.reason = "Retreat: Moving away from enemy";
        }
    }
    
    return decision;
}

DecisionEngine::Decision DecisionEngine::executeRepositionStrategy(
    const GameState& game_state, const GoalManager& goal_manager) {
    
    Decision decision;
    decision.strategy = Strategy::REPOSITION;
    
    // 重新定位策略：选择当前状态下的最佳位置
    auto goals = goal_manager.getGoalsByPriority(4);
    
    if (!goals.empty()) {
        // 从中选择评分最高的
        double max_score = -1.0;
        int best_goal_id = -1;
        
        for (int goal_id : goals) {
            double score = calculateGoalScore(goal_id, game_state, goal_manager);
            if (score > max_score) {
                max_score = score;
                best_goal_id = goal_id;
            }
        }
        
        if (best_goal_id >= 0) {
            decision.goal_id = best_goal_id;
            goal_manager.getGoalPoint(decision.goal_id, decision.goal_pose);
            decision.confidence = 0.75;
            decision.reason = "Reposition: Relocating to better position";
        }
    }
    
    return decision;
}

int DecisionEngine::selectBestGoalByPriority(
    const GameState& game_state, const GoalManager& goal_manager) {
    
    auto goals = goal_manager.getGoalsByPriority();
    if (!goals.empty()) {
        return goals[0];
    }
    
    auto available = goal_manager.getAvailableGoals();
    if (!available.empty()) {
        return available[0];
    }
    
    return -1;
}

int DecisionEngine::selectGoalNearBuilding(
    const GameState& game_state, const GoalManager& goal_manager, 
    GameState::BuildingType building_type) {
    
    // 简化实现：选择高优先级目标
    // 实际应该根据建筑位置选择最近的目标点
    auto goals = goal_manager.getGoalsByPriority(5);
    if (!goals.empty()) {
        return goals[0];
    }
    return -1;
}

int DecisionEngine::selectGoalAwayFromEnemy(
    const GameState& game_state, const GoalManager& goal_manager) {
    
    if (!game_state.isEnemyVisible()) {
        // 如果敌人不可见，选择随机安全位置
        auto goals = goal_manager.getAvailableGoals();
        if (!goals.empty()) {
            return goals[0];
        }
        return -1;
    }
    
    double enemy_x, enemy_y, enemy_z;
    game_state.getEnemyPosition(enemy_x, enemy_y, enemy_z);
    
    auto goals = goal_manager.getAvailableGoals();
    double max_distance = -1.0;
    int best_goal_id = -1;
    
    for (int goal_id : goals) {
        geometry_msgs::PoseStamped pose;
        if (goal_manager.getGoalPoint(goal_id, pose)) {
            double dx = pose.pose.position.x - enemy_x;
            double dy = pose.pose.position.y - enemy_y;
            double distance = std::sqrt(dx*dx + dy*dy);
            
            if (distance > max_distance) {
                max_distance = distance;
                best_goal_id = goal_id;
            }
        }
    }
    
    return best_goal_id;
}

double DecisionEngine::calculateGoalScore(
    int goal_id, const GameState& game_state, const GoalManager& goal_manager) {
    
    geometry_msgs::PoseStamped pose;
    if (!goal_manager.getGoalPoint(goal_id, pose)) {
        return 0.0;
    }
    
    double score = 0.0;
    
    // 血量因素
    double hp_percent = game_state.getSelfHPPercent();
    score += hp_percent * weights_.hp_weight;
    
    // 弹丸因素
    double ammo_percent = game_state.getSelfAmmoPercent();
    score += ammo_percent * weights_.ammo_weight;
    
    // 建筑因素
    double base_hp_percent = game_state.getBuildingHPPercent(
        GameState::BuildingType::BASE, true);
    double outpost_hp_percent = game_state.getBuildingHPPercent(
        GameState::BuildingType::SENTRY_OUTPOST, true);
    double building_score = (base_hp_percent + outpost_hp_percent) / 2.0;
    score += building_score * weights_.building_weight;
    
    // 敌人因素（如果可见）
    if (game_state.isEnemyVisible()) {
        double enemy_x, enemy_y, enemy_z;
        game_state.getEnemyPosition(enemy_x, enemy_y, enemy_z);
        
        double dx = pose.pose.position.x - enemy_x;
        double dy = pose.pose.position.y - enemy_y;
        double distance = std::sqrt(dx*dx + dy*dy);
        // 根据策略调整距离分数（这里简化为适中距离更好）
        double distance_score = 1.0 / (1.0 + distance / 5.0);
        score += distance_score * weights_.enemy_weight;
    }
    
    return std::min(score, 1.0);
}

void DecisionEngine::setDefaultStrategy(Strategy strategy) {
    default_strategy_ = strategy;
}

DecisionEngine::Strategy DecisionEngine::getDefaultStrategy() const {
    return default_strategy_;
}

void DecisionEngine::setStrategyWeights(const StrategyWeights& weights) {
    weights_ = weights;
}

} // namespace goal_distribute
