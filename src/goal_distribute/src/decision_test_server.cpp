/**
 * @file decision_test_server.cpp
 * @brief 决策测试服务节点
 * 
 * 提供ROS服务接口，接收游戏状态并返回决策结果
 * 用于前端测试界面
 */

#include "goal_distribute/game_state.hpp"
#include "goal_distribute/goal_manager.hpp"
#include "goal_distribute/decision_engine.hpp"
#include "goal_distribute/DecisionTest.h"

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>

class DecisionTestServer {
public:
    DecisionTestServer() : nh_("~") {
        // 初始化游戏状态
        game_state_.setTeamColor("red");
        
        // 加载目标点配置（从ROS参数服务器）
        // 注意：需要先在参数服务器中配置目标点，或通过launch文件加载
        goal_manager_.loadGoalsFromROSParam(nh_, game_state_.getTeamColor());
        
        // 如果没有目标点，添加一些默认目标点用于测试
        if (goal_manager_.getAvailableGoals().empty()) {
            ROS_WARN("未找到目标点配置，添加默认测试目标点");
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.orientation.w = 1.0;
            
            pose.pose.position.x = 0.0; pose.pose.position.y = 0.0;
            goal_manager_.addGoalPoint(0, pose, "base_defense", 5);
            
            pose.pose.position.x = 1.0; pose.pose.position.y = 1.0;
            goal_manager_.addGoalPoint(1, pose, "outpost_defense", 4);
            
            pose.pose.position.x = 2.0; pose.pose.position.y = 2.0;
            goal_manager_.addGoalPoint(2, pose, "forward_position", 3);
        }
        
        // 设置决策引擎默认策略
        decision_engine_.setDefaultStrategy(
            goal_distribute::DecisionEngine::Strategy::BALANCED);
        
        // 加载JSON规则配置（如果存在）
        std::string package_path = ros::package::getPath("goal_distribute");
        std::string default_json_path = package_path + "/config/decision_rules.json";
        if (decision_engine_.loadRulesFromJSON(default_json_path)) {
            decision_engine_.setUseJSONRules(true);
            ROS_INFO("决策测试服务器: 已加载JSON规则配置");
        } else {
            ROS_INFO("决策测试服务器: 使用传统决策方法");
            decision_engine_.setUseJSONRules(false);
        }
        
        // 创建服务
        service_ = nh_.advertiseService("decision_test", 
                                       &DecisionTestServer::decisionCallback, this);
        
        ROS_INFO("决策测试服务器已启动，服务名称: /goal_distribute/decision_test");
    }
    
private:
    /**
     * @brief 决策服务回调函数
     * @param req 请求：游戏状态
     * @param res 响应：决策结果
     * @return 是否成功处理请求
     */
    bool decisionCallback(goal_distribute::DecisionTest::Request& req,
                         goal_distribute::DecisionTest::Response& res) {
        try {
            // 设置队伍阵营（默认红方）
            std::string team_color = req.team_color;
            if (team_color != "red" && team_color != "blue") {
                team_color = "red";
            }
            if (team_color != game_state_.getTeamColor()) {
                game_state_.setTeamColor(team_color);
            }

            // 更新游戏状态
            game_state_.setSelfHP(req.self_hp);
            game_state_.setSelfAmmo(req.self_ammo);
            
            // 设置建筑血量
            std::map<goal_distribute::GameState::BuildingType, uint16_t> ally_buildings;
            std::map<goal_distribute::GameState::BuildingType, uint16_t> enemy_buildings;
            
            ally_buildings[goal_distribute::GameState::BuildingType::BASE] = req.ally_base_hp;
            ally_buildings[goal_distribute::GameState::BuildingType::SENTRY_OUTPOST] = req.ally_outpost_hp;
            
            enemy_buildings[goal_distribute::GameState::BuildingType::BASE] = req.enemy_base_hp;
            enemy_buildings[goal_distribute::GameState::BuildingType::SENTRY_OUTPOST] = req.enemy_outpost_hp;
            
            game_state_.setAllBuildingHP(ally_buildings, enemy_buildings);
            
            // 设置敌人状态
            if (req.enemy_visible) {
                game_state_.setEnemyPosition(req.enemy_x, req.enemy_y, req.enemy_z);
            } else {
                game_state_.setEnemyVisible(false);
            }
            
            // 执行决策
            auto decision = decision_engine_.makeDecision(game_state_, goal_manager_);
            
            // 填充响应
            res.goal_id = decision.goal_id;
            res.goal_pose = decision.goal_pose;
            
            // 转换策略枚举为字符串
            switch (decision.strategy) {
            case goal_distribute::DecisionEngine::Strategy::DEFENSIVE:
                res.strategy = "DEFENSIVE"; break;
            case goal_distribute::DecisionEngine::Strategy::AGGRESSIVE:
                res.strategy = "AGGRESSIVE"; break;
            case goal_distribute::DecisionEngine::Strategy::BALANCED:
                res.strategy = "BALANCED"; break;
            case goal_distribute::DecisionEngine::Strategy::RETREAT:
                res.strategy = "RETREAT"; break;
            case goal_distribute::DecisionEngine::Strategy::REPOSITION:
                res.strategy = "REPOSITION"; break;
            case goal_distribute::DecisionEngine::Strategy::SUPPLY_RETURN:
                res.strategy = "SUPPLY_RETURN"; break;
            case goal_distribute::DecisionEngine::Strategy::SUPPLY_GUARD:
                res.strategy = "SUPPLY_GUARD"; break;
            case goal_distribute::DecisionEngine::Strategy::ATTACK_ENEMY_BASE:
                res.strategy = "ATTACK_ENEMY_BASE"; break;
            case goal_distribute::DecisionEngine::Strategy::ATTACK_ENEMY_OUTPOST:
                res.strategy = "ATTACK_ENEMY_OUTPOST"; break;
            case goal_distribute::DecisionEngine::Strategy::AGGRESSIVE_AIM:
                res.strategy = "AGGRESSIVE_AIM"; break;
            case goal_distribute::DecisionEngine::Strategy::CONSERVATIVE_AIM:
                res.strategy = "CONSERVATIVE_AIM"; break;
                default:
                res.strategy = "UNKNOWN"; break;
            }
            
            res.reason = decision.reason;
            res.confidence = decision.confidence;
            res.success = (decision.goal_id >= 0);
            
            ROS_DEBUG("决策测试: goal_id=%d, strategy=%s, confidence=%.2f", 
                     decision.goal_id, res.strategy.c_str(), res.confidence);
            
            return true;
            
        } catch (const std::exception& e) {
            ROS_ERROR("决策测试服务错误: %s", e.what());
            res.success = false;
            res.reason = std::string("错误: ") + e.what();
            return false;
        }
    }
    
    ros::NodeHandle nh_;
    goal_distribute::GameState game_state_;
    goal_distribute::GoalManager goal_manager_;
    goal_distribute::DecisionEngine decision_engine_;
    ros::ServiceServer service_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "decision_test_server");
    
    try {
        DecisionTestServer server;
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("决策测试服务器启动失败: %s", e.what());
        return -1;
    }
    
    return 0;
}

