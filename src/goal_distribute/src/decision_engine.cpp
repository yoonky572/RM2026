/**
 * @file decision_engine.cpp
 * @brief 决策引擎实现
 * 
 * 核心决策逻辑：
 * 1. 支持基于JSON规则的决策（推荐）
 * 2. 支持传统的if-else决策方法（备用）
 * 3. 根据游戏状态（血量、弹丸、建筑状态、敌人位置）做出最优决策
 */

#include "goal_distribute/decision_engine.hpp"
#include <algorithm>
#include <cmath>
#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <sstream>

// 使用nlohmann/json库（header-only）解析JSON规则配置
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace goal_distribute {

/**
 * @brief 决策引擎构造函数
 * 
 * 初始化默认策略和权重参数
 * 默认使用平衡策略（BALANCED）
 */
namespace {

using Strategy = goal_distribute::DecisionEngine::Strategy;

Strategy strategyFromString(const std::string& name) {
    if (name == "SUPPLY_RETURN") return Strategy::SUPPLY_RETURN;
    if (name == "SUPPLY_GUARD") return Strategy::SUPPLY_GUARD;
    if (name == "ATTACK_ENEMY_BASE") return Strategy::ATTACK_ENEMY_BASE;
    if (name == "ATTACK_ENEMY_OUTPOST") return Strategy::ATTACK_ENEMY_OUTPOST;
    if (name == "AGGRESSIVE_AIM") return Strategy::AGGRESSIVE_AIM;
    if (name == "CONSERVATIVE_AIM") return Strategy::CONSERVATIVE_AIM;
    if (name == "DEFENSIVE") return Strategy::DEFENSIVE;
    if (name == "AGGRESSIVE") return Strategy::AGGRESSIVE;
    if (name == "BALANCED") return Strategy::BALANCED;
    if (name == "RETREAT") return Strategy::RETREAT;
    if (name == "REPOSITION") return Strategy::REPOSITION;
    return Strategy::UNKNOWN;
}

const char* strategyToString(Strategy strategy) {
    switch (strategy) {
        case Strategy::SUPPLY_RETURN: return "SUPPLY_RETURN";
        case Strategy::SUPPLY_GUARD: return "SUPPLY_GUARD";
        case Strategy::ATTACK_ENEMY_BASE: return "ATTACK_ENEMY_BASE";
        case Strategy::ATTACK_ENEMY_OUTPOST: return "ATTACK_ENEMY_OUTPOST";
        case Strategy::AGGRESSIVE_AIM: return "AGGRESSIVE_AIM";
        case Strategy::CONSERVATIVE_AIM: return "CONSERVATIVE_AIM";
        case Strategy::DEFENSIVE: return "DEFENSIVE";
        case Strategy::AGGRESSIVE: return "AGGRESSIVE";
        case Strategy::BALANCED: return "BALANCED";
        case Strategy::RETREAT: return "RETREAT";
        case Strategy::REPOSITION: return "REPOSITION";
        default: return "UNKNOWN";
    }
}

} // namespace

DecisionEngine::DecisionEngine()
    : default_strategy_(Strategy::BALANCED)
    , use_json_rules_(false)
{
    // 初始化评分权重（用于best_score类型的目标选择）
    weights_.hp_weight = 0.3;        // 血量权重
    weights_.ammo_weight = 0.2;     // 弹丸权重
    weights_.building_weight = 0.3; // 建筑状态权重
    weights_.enemy_weight = 0.2;    // 敌人位置权重
}

/**
 * @brief 主要决策接口
 * 
 * 根据游戏状态和目标点管理器做出决策
 * 优先使用JSON规则（如果已加载），否则使用传统方法
 * 
 * @param game_state 游戏状态（血量、弹丸、建筑状态等）
 * @param goal_manager 目标点管理器
 * @return Decision 决策结果（目标点ID、位置、策略、原因、置信度）
 */
DecisionEngine::Decision DecisionEngine::makeDecision(
    const GameState& game_state, const GoalManager& goal_manager) {
    
    // 优先使用JSON规则（如果已加载且启用）
    if (use_json_rules_ && !json_rules_.empty()) {
        return makeDecisionFromJSONRules(game_state, goal_manager);
    }
    
    // 否则使用传统的if-else决策方法
    Decision decision;
    decision.strategy = Strategy::UNKNOWN;
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

bool DecisionEngine::loadRulesFromJSON(const std::string& json_file_path) {
    try {
        std::ifstream file(json_file_path);
        if (!file.is_open()) {
            ROS_ERROR("Failed to open JSON file: %s", json_file_path.c_str());
            return false;
        }
        
        json j;
        file >> j;
        file.close();
        
        // 清空现有规则
        json_rules_.clear();
        
        // 解析权重（如果存在）
        if (j.contains("weights")) {
            auto weights_json = j["weights"];
            if (weights_json.contains("hp_weight")) {
                weights_.hp_weight = weights_json["hp_weight"];
            }
            if (weights_json.contains("ammo_weight")) {
                weights_.ammo_weight = weights_json["ammo_weight"];
            }
            if (weights_json.contains("building_weight")) {
                weights_.building_weight = weights_json["building_weight"];
            }
            if (weights_json.contains("enemy_weight")) {
                weights_.enemy_weight = weights_json["enemy_weight"];
            }
        }
        
        // 解析规则
        if (!j.contains("rules") || !j["rules"].is_array()) {
            ROS_ERROR("JSON file must contain a 'rules' array");
            return false;
        }
        
        for (const auto& rule_json : j["rules"]) {
            auto rule = std::make_unique<RuleConfig>();
            
            // 解析规则名称和优先级
            rule->name = rule_json.value("name", "");
            rule->priority = rule_json.value("priority", 0);
            
            // 解析条件
            if (rule_json.contains("conditions")) {
                auto conditions_json = rule_json["conditions"];
                
                // 解析数值条件
                for (auto it = conditions_json.begin(); it != conditions_json.end(); ++it) {
                    std::string key = it.key();
                    if (it.value().is_object() && it.value().contains("operator") && it.value().contains("value")) {
                        ConditionValue cond_val;
                        cond_val.operator_ = it.value()["operator"];
                        cond_val.value = it.value()["value"];
                        rule->conditions.numeric_conditions[key] = cond_val;
                    } else if (it.value().is_boolean()) {
                        // 布尔条件
                        rule->conditions.bool_conditions[key] = it.value();
                    }
                }
            }
            
            // 解析动作
            if (rule_json.contains("action")) {
                auto action_json = rule_json["action"];
                rule->action.strategy = action_json.value("strategy", "BALANCED");
                rule->action.confidence = action_json.value("confidence", 0.5);
                rule->action.reason = action_json.value("reason", "");
                
                // 解析目标选择配置
                if (action_json.contains("goal_selection")) {
                    auto goal_sel_json = action_json["goal_selection"];
                    rule->action.goal_selection.type = goal_sel_json.value("type", "best_score");
                    rule->action.goal_selection.building_type = goal_sel_json.value("building_type", "");
                    rule->action.goal_selection.ideal_distance = goal_sel_json.value("ideal_distance", 3.0);
                    rule->action.goal_selection.min_priority = goal_sel_json.value("min_priority", 0);
                }
            }
            
            json_rules_.push_back(std::move(rule));
        }
        
        // 按优先级排序（优先级高的在前）
        std::sort(json_rules_.begin(), json_rules_.end(),
                  [](const std::unique_ptr<RuleConfig>& a, const std::unique_ptr<RuleConfig>& b) {
                      return a->priority > b->priority;
                  });
        
        ROS_INFO("Loaded %zu rules from JSON file: %s", json_rules_.size(), json_file_path.c_str());
        return true;
        
    } catch (const json::exception& e) {
        ROS_ERROR("JSON parsing error: %s", e.what());
        return false;
    } catch (const std::exception& e) {
        ROS_ERROR("Error loading JSON rules: %s", e.what());
        return false;
    }
}

void DecisionEngine::setUseJSONRules(bool enable) {
    use_json_rules_ = enable;
    if (enable && json_rules_.empty()) {
        ROS_WARN("JSON rules enabled but no rules loaded. Use loadRulesFromJSON() first.");
    }
}

DecisionEngine::Decision DecisionEngine::makeDecisionFromJSONRules(
    const GameState& game_state, const GoalManager& goal_manager) {
    
    Decision decision;
    decision.strategy = Strategy::UNKNOWN;
    decision.confidence = 0.0;
    decision.goal_id = -1;
    
    // 按优先级顺序检查规则
    for (const auto& rule : json_rules_) {
        if (checkConditions(rule->conditions, game_state)) {
            // 找到第一个满足条件的规则，执行其动作
            decision = executeAction(rule->action, game_state, goal_manager);
            ROS_DEBUG("Matched rule: %s (priority: %d)", rule->name.c_str(), rule->priority);
            return decision;
        }
    }
    
    // 如果没有规则匹配，使用默认策略
    ROS_WARN("No JSON rule matched, using fallback strategy");
    decision.goal_id = selectBestGoalByPriority(game_state, goal_manager);
    if (decision.goal_id >= 0) {
        goal_manager.getGoalPoint(decision.goal_id, decision.goal_pose);
        decision.confidence = 0.5;
        decision.reason = "Fallback: No rule matched";
        decision.strategy = Strategy::CONSERVATIVE_AIM;
    }
    
    return decision;
}

bool DecisionEngine::checkConditions(const ConditionConfig& conditions, const GameState& game_state) {
    // 检查数值条件
    for (const auto& cond : conditions.numeric_conditions) {
        const std::string& key = cond.first;
        const ConditionValue& cond_val = cond.second;
        
        double actual_value = 0.0;
        
        if (key == "self_hp_percent") {
            actual_value = game_state.getSelfHPPercent();
        } else if (key == "self_ammo_percent") {
            actual_value = game_state.getSelfAmmoPercent();
        } else if (key == "base_hp_percent") {
            actual_value = game_state.getBuildingHPPercent(GameState::BuildingType::BASE, true);
        } else if (key == "outpost_hp_percent") {
            actual_value = game_state.getBuildingHPPercent(GameState::BuildingType::SENTRY_OUTPOST, true);
        } else if (key == "enemy_base_hp_percent") {
            actual_value = game_state.getBuildingHPPercent(GameState::BuildingType::BASE, false);
        } else if (key == "enemy_outpost_hp_percent") {
            actual_value = game_state.getBuildingHPPercent(GameState::BuildingType::SENTRY_OUTPOST, false);
        } else {
            ROS_WARN("Unknown numeric condition key: %s", key.c_str());
            continue;
        }
        
        // 根据操作符比较
        bool result = false;
        if (cond_val.operator_ == "lt") {
            result = actual_value < cond_val.value;
        } else if (cond_val.operator_ == "lte") {
            result = actual_value <= cond_val.value;
        } else if (cond_val.operator_ == "gt") {
            result = actual_value > cond_val.value;
        } else if (cond_val.operator_ == "gte") {
            result = actual_value >= cond_val.value;
        } else if (cond_val.operator_ == "eq") {
            result = std::abs(actual_value - cond_val.value) < 1e-6;
        } else if (cond_val.operator_ == "ne") {
            result = std::abs(actual_value - cond_val.value) >= 1e-6;
        } else {
            ROS_WARN("Unknown operator: %s", cond_val.operator_.c_str());
            continue;
        }
        
        if (!result) {
            return false;  // 条件不满足
        }
    }
    
    // 检查布尔条件
    for (const auto& cond : conditions.bool_conditions) {
        const std::string& key = cond.first;
        bool expected_value = cond.second;
        
        bool actual_value = false;
        
        if (key == "enemy_visible") {
            actual_value = game_state.isEnemyVisible();
        } else {
            ROS_WARN("Unknown boolean condition key: %s", key.c_str());
            continue;
        }
        
        if (actual_value != expected_value) {
            return false;  // 条件不满足
        }
    }
    
    return true;  // 所有条件都满足
}

DecisionEngine::Decision DecisionEngine::executeAction(
    const ActionConfig& action, const GameState& game_state, const GoalManager& goal_manager) {
    
    Decision decision;
    decision.strategy = Strategy::UNKNOWN;
    decision.confidence = action.confidence;
    decision.reason = action.reason;
    decision.goal_id = -1;
    
    // 转换策略字符串为枚举
    decision.strategy = strategyFromString(action.strategy);
    if (decision.strategy == Strategy::UNKNOWN) {
        ROS_WARN("Unknown strategy: %s, fallback to CONSERVATIVE_AIM", action.strategy.c_str());
        decision.strategy = Strategy::CONSERVATIVE_AIM;
    }
    
    // 根据目标选择类型选择目标
    const auto& goal_sel = action.goal_selection;
    
    if (goal_sel.type == "near_building") {
        GameState::BuildingType building_type;
        if (goal_sel.building_type == "BASE") {
            building_type = GameState::BuildingType::BASE;
        } else if (goal_sel.building_type == "SENTRY_OUTPOST") {
            building_type = GameState::BuildingType::SENTRY_OUTPOST;
        } else {
            ROS_WARN("Unknown building type: %s", goal_sel.building_type.c_str());
            building_type = GameState::BuildingType::BASE;
        }
        decision.goal_id = selectGoalNearBuilding(game_state, goal_manager, building_type);
    } else if (goal_sel.type == "near_enemy") {
        decision.goal_id = selectGoalNearEnemy(game_state, goal_manager, goal_sel.ideal_distance);
    } else if (goal_sel.type == "away_from_enemy") {
        decision.goal_id = selectGoalAwayFromEnemy(game_state, goal_manager);
    } else if (goal_sel.type == "best_score") {
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
        decision.goal_id = best_goal_id;
    } else if (goal_sel.type == "by_priority") {
        auto goals = goal_manager.getGoalsByPriority(goal_sel.min_priority);
        if (!goals.empty()) {
            decision.goal_id = goals[0];
        }
    } else {
        ROS_WARN("Unknown goal selection type: %s, using best_score", goal_sel.type.c_str());
        // 默认使用best_score
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
        decision.goal_id = best_goal_id;
    }
    
    // 获取目标点位置
    if (decision.goal_id >= 0) {
        if (!goal_manager.getGoalPoint(decision.goal_id, decision.goal_pose)) {
            ROS_WARN("Failed to get goal point for goal_id: %d", decision.goal_id);
            decision.goal_id = -1;
        }
    }
    
    return decision;
}

int DecisionEngine::selectGoalNearEnemy(
    const GameState& game_state, const GoalManager& goal_manager, double ideal_distance) {
    
    if (!game_state.isEnemyVisible()) {
        // 如果敌人不可见，使用best_score
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
        return best_goal_id;
    }
    
    double enemy_x, enemy_y, enemy_z;
    game_state.getEnemyPosition(enemy_x, enemy_y, enemy_z);
    
    auto goals = goal_manager.getAvailableGoals();
    double best_score = -1.0;
    int best_goal_id = -1;
    
    for (int goal_id : goals) {
        geometry_msgs::PoseStamped pose;
        if (goal_manager.getGoalPoint(goal_id, pose)) {
            double score = calculateGoalScore(goal_id, game_state, goal_manager);
            
            // 计算距离分数（距离理想距离越近越好）
            double dx = pose.pose.position.x - enemy_x;
            double dy = pose.pose.position.y - enemy_y;
            double distance = std::sqrt(dx*dx + dy*dy);
            double distance_score = 1.0 / (1.0 + std::abs(distance - ideal_distance));
            
            // 综合评分
            score = score * 0.6 + distance_score * 0.4;
            
            if (score > best_score) {
                best_score = score;
                best_goal_id = goal_id;
            }
        }
    }
    
    return best_goal_id;
}

} // namespace goal_distribute
