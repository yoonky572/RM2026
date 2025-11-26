/**
 * @file decision_engine.cpp
 * @brief 决策引擎实现（简化版，只返回策略对应的固定ID）
 * 
 * 核心决策逻辑：
 * 1. 基于JSON规则的决策
 * 2. 根据游戏状态匹配规则
 * 3. 返回策略对应的固定目标点ID（0-10）
 */

#include "goal_distribute/decision_engine.hpp"
#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <filesystem>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace goal_distribute {

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

/**
 * @brief 策略到固定ID的映射（主键思路）
 */
int fixedGoalIdForStrategy(Strategy strategy) {
    switch (strategy) {
        case Strategy::SUPPLY_RETURN: return 0;
        case Strategy::SUPPLY_GUARD: return 1;
        case Strategy::ATTACK_ENEMY_BASE: return 2;
        case Strategy::ATTACK_ENEMY_OUTPOST: return 3;
        case Strategy::AGGRESSIVE_AIM: return 4;
        case Strategy::CONSERVATIVE_AIM: return 5;
        case Strategy::DEFENSIVE: return 6;
        case Strategy::AGGRESSIVE: return 7;
        case Strategy::BALANCED: return 8;
        case Strategy::RETREAT: return 9;
        case Strategy::REPOSITION: return 10;
        default: return -1;
    }
}

std::vector<std::string> collectJSONFiles(const std::string& path_string) {
    namespace fs = std::filesystem;
    std::vector<std::string> files;
    fs::path target(path_string);
    
    if (!fs::exists(target)) {
        return files;
    }
    
    if (fs::is_directory(target)) {
        for (const auto& entry : fs::directory_iterator(target)) {
            if (entry.is_regular_file() && entry.path().extension() == ".json") {
                files.emplace_back(entry.path().string());
            }
        }
        std::sort(files.begin(), files.end());
    } else if (fs::is_regular_file(target) && target.extension() == ".json") {
        files.emplace_back(target.string());
    }
    
    return files;
}

} // namespace

DecisionEngine::DecisionEngine()
    : default_strategy_(Strategy::BALANCED)
    , use_json_rules_(false)
{
}

/**
 * @brief 主要决策接口
 * 
 * 根据游戏状态做出决策，返回策略对应的固定ID
 * 
 * @param game_state 游戏状态（血量、弹丸、建筑状态等）
 * @return Decision 决策结果（目标点ID、策略、原因、置信度）
 */
DecisionEngine::Decision DecisionEngine::makeDecision(const GameState& game_state) {
    Decision decision;
    decision.strategy = Strategy::UNKNOWN;
    decision.confidence = 0.0;
    decision.goal_id = -1;
    
    if (json_rules_.empty()) {
        ROS_ERROR_THROTTLE(1.0, "JSON rules are required but not loaded.");
        decision.reason = "JSON rules unavailable";
        return decision;
    }
    
    return makeDecisionFromJSONRules(game_state);
}

bool DecisionEngine::loadRulesFromJSON(const std::string& json_file_path) {
    const auto json_files = collectJSONFiles(json_file_path);
    if (json_files.empty()) {
        ROS_ERROR("No JSON files found at path: %s", json_file_path.c_str());
        return false;
    }
    
    auto parse_rules = [&](const json& j, const std::string& source_name) -> bool {
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
        
        if (!j.contains("rules") || !j["rules"].is_array()) {
            ROS_ERROR("JSON file %s must contain a 'rules' array", source_name.c_str());
            return false;
        }
        
        for (const auto& rule_json : j["rules"]) {
            auto rule = std::make_unique<RuleConfig>();
            rule->name = rule_json.value("name", "");
            rule->priority = rule_json.value("priority", 0);
            
            if (rule_json.contains("conditions")) {
                auto conditions_json = rule_json["conditions"];
                for (auto it = conditions_json.begin(); it != conditions_json.end(); ++it) {
                    std::string key = it.key();
                    if (it.value().is_object() && it.value().contains("operator") && it.value().contains("value")) {
                        ConditionValue cond_val;
                        cond_val.operator_ = it.value()["operator"];
                        cond_val.value = it.value()["value"];
                        rule->conditions.numeric_conditions[key] = cond_val;
                    } else if (it.value().is_boolean()) {
                        rule->conditions.bool_conditions[key] = it.value();
                    }
                }
            }
            
            if (rule_json.contains("action")) {
                auto action_json = rule_json["action"];
                rule->action.strategy = action_json.value("strategy", "BALANCED");
                rule->action.confidence = action_json.value("confidence", 0.5);
                rule->action.reason = action_json.value("reason", "");
                
                if (action_json.contains("goal_selection")) {
                    auto goal_sel_json = action_json["goal_selection"];
                    rule->action.goal_selection.type = goal_sel_json.value("type", "fixed_id");
                    rule->action.goal_selection.goal_id = goal_sel_json.value("goal_id", -1);
                } else {
                    rule->action.goal_selection.goal_id = -1;
                }
            }
            
            json_rules_.push_back(std::move(rule));
        }
        
        return true;
    };
    
    json_rules_.clear();
    
    try {
        for (const auto& file_path : json_files) {
            std::ifstream file(file_path);
            if (!file.is_open()) {
                ROS_ERROR("Failed to open JSON file: %s", file_path.c_str());
                json_rules_.clear();
                return false;
            }
            
            json j;
            file >> j;
            file.close();
            
            if (!parse_rules(j, file_path)) {
                json_rules_.clear();
                return false;
            }
        }
        
        if (json_rules_.empty()) {
            ROS_ERROR("No valid rules found in path: %s", json_file_path.c_str());
            return false;
        }
        
        std::sort(json_rules_.begin(), json_rules_.end(),
                  [](const std::unique_ptr<RuleConfig>& a, const std::unique_ptr<RuleConfig>& b) {
                      return a->priority > b->priority;
                  });
        
        ROS_INFO("Loaded %zu rules from %zu JSON file(s) under: %s",
                 json_rules_.size(), json_files.size(), json_file_path.c_str());
        return true;
        
    } catch (const json::exception& e) {
        ROS_ERROR("JSON parsing error: %s", e.what());
        json_rules_.clear();
        return false;
    } catch (const std::exception& e) {
        ROS_ERROR("Error loading JSON rules: %s", e.what());
        json_rules_.clear();
        return false;
    }
}

void DecisionEngine::setUseJSONRules(bool enable) {
    if (!enable) {
        ROS_WARN("Traditional decision flow has been removed; JSON rules remain enabled.");
        enable = true;
    }
    
    use_json_rules_ = enable;
    if (json_rules_.empty()) {
        ROS_WARN("JSON rules mode active but no rules loaded. Call loadRulesFromJSON() first.");
    }
}

DecisionEngine::Decision DecisionEngine::makeDecisionFromJSONRules(const GameState& game_state) {
    Decision decision;
    decision.strategy = Strategy::UNKNOWN;
    decision.confidence = 0.0;
    decision.goal_id = -1;
    
    // 按优先级顺序检查规则
    for (const auto& rule : json_rules_) {
        if (checkConditions(rule->conditions, game_state)) {
            // 找到第一个满足条件的规则，执行其动作
            decision = executeAction(rule->action, game_state);
            ROS_DEBUG("Matched rule: %s (priority: %d)", rule->name.c_str(), rule->priority);
            return decision;
        }
    }
    
    // 如果没有规则匹配，使用默认策略
    ROS_WARN("No JSON rule matched, using fallback strategy");
    decision.strategy = Strategy::CONSERVATIVE_AIM;
    decision.goal_id = fixedGoalIdForStrategy(decision.strategy);
    decision.confidence = 0.5;
    decision.reason = "Fallback: No rule matched";
    
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
    const ActionConfig& action, const GameState& game_state) {
    
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
    
    // 优先使用JSON中显式指定的goal_id，否则使用策略固定映射
    const auto& goal_sel = action.goal_selection;
    if (goal_sel.goal_id >= 0) {
        decision.goal_id = goal_sel.goal_id;
    } else {
        decision.goal_id = fixedGoalIdForStrategy(decision.strategy);
    }
    
    if (decision.goal_id < 0) {
        ROS_WARN("Failed to determine goal_id for strategy: %s", action.strategy.c_str());
        decision.goal_id = fixedGoalIdForStrategy(Strategy::CONSERVATIVE_AIM);
    }
    
    return decision;
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
