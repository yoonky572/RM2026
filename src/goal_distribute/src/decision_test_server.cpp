/**
 * @file decision_test_server.cpp
 * @brief 决策测试服务节点（简化版，只返回策略ID）
 * 
 * 提供ROS服务接口，接收游戏状态并返回决策结果
 * 只返回策略对应的固定ID，不包含位置信息
 */

#include "goal_distribute/game_state.hpp"
#include "goal_distribute/goal_manager.hpp"
#include "goal_distribute/decision_engine.hpp"
#include "goal_distribute/DecisionTest.h"

#include <ros/ros.h>
#include <ros/package.h>
#include <vector>
#include <string>

class DecisionTestServer {
public:
    DecisionTestServer() : nh_("~") {
        ROS_INFO("决策测试服务器: 开始初始化...");
        
        // 立即注册服务，确保bridge可以连接
        ROS_INFO("决策测试服务器: 正在注册ROS服务...");
        ros::NodeHandle public_nh;
        service_ = public_nh.advertiseService("goal_distribute/decision_test", 
                                             &DecisionTestServer::decisionCallback, this);
        // 立即处理一次，确保服务注册到master
        ros::spinOnce();
        ROS_INFO("决策测试服务器: ROS服务已注册: /goal_distribute/decision_test");
        
        // 初始化游戏状态
        game_state_.setTeamColor("red");
        ROS_INFO("决策测试服务器: 游戏状态已初始化");
        
        // 设置决策引擎默认策略
        decision_engine_.setDefaultStrategy(
            goal_distribute::DecisionEngine::Strategy::BALANCED);
        ROS_INFO("决策测试服务器: 默认策略已设置");
        
        // 初始化目标点ID管理器（确保所有固定ID都存在）
        std::vector<int> goal_ids = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
        goal_manager_.addGoalIds(goal_ids);
        ROS_INFO("决策测试服务器: 目标点ID已初始化（共 %zu 个）", goal_manager_.getGoalCount());
        
        // 加载JSON规则配置（如果存在）- 在服务注册之后进行，避免阻塞
        std::string json_config_path;
        if (nh_.getParam("json_rules_file", json_config_path)) {
            ROS_INFO("决策测试服务器: 从参数加载JSON规则: %s", json_config_path.c_str());
            if (decision_engine_.loadRulesFromJSON(json_config_path)) {
                decision_engine_.setUseJSONRules(true);
                ROS_INFO("决策测试服务器: JSON规则配置已加载");
            } else {
                ROS_WARN("决策测试服务器: JSON规则加载失败");
                decision_engine_.setUseJSONRules(false);
            }
        } else {
            // 尝试使用默认路径
            ROS_INFO("决策测试服务器: 尝试加载默认JSON规则...");
            try {
                std::string package_path = ros::package::getPath("goal_distribute");
                std::string default_json_path = package_path + "/config/decision_rules.json";
                if (decision_engine_.loadRulesFromJSON(default_json_path)) {
                    decision_engine_.setUseJSONRules(true);
                    ROS_INFO("决策测试服务器: 默认JSON规则配置已加载: %s", default_json_path.c_str());
                } else {
                    ROS_WARN("决策测试服务器: 未找到JSON规则文件");
                    decision_engine_.setUseJSONRules(false);
                }
            } catch (const std::exception& e) {
                ROS_WARN("决策测试服务器: 获取包路径失败: %s，跳过JSON规则加载", e.what());
                decision_engine_.setUseJSONRules(false);
            }
        }
        
        ROS_INFO("决策测试服务器已完全启动，服务名称: /goal_distribute/decision_test");
    }
    
private:
    /**
     * @brief 决策服务回调函数
     * @param req 请求：游戏状态
     * @param res 响应：决策结果（只包含ID，不包含位置）
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
            
            // 执行决策（不再需要GoalManager）
            auto decision = decision_engine_.makeDecision(game_state_);
            
            // 填充响应（只返回ID，不返回位置）
            res.goal_id = decision.goal_id;
            // goal_pose 字段保留但留空（兼容性）
            res.goal_pose.header.frame_id = "map";
            res.goal_pose.pose.position.x = 0.0;
            res.goal_pose.pose.position.y = 0.0;
            res.goal_pose.pose.position.z = 0.0;
            res.goal_pose.pose.orientation.w = 1.0;
            
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
    goal_distribute::GoalManager goal_manager_;  // 只用于ID管理，不管理位置
    goal_distribute::DecisionEngine decision_engine_;
    ros::ServiceServer service_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "decision_test_server");
    
    // 确保ROS节点完全初始化
    if (!ros::ok()) {
        ROS_ERROR("ROS节点初始化失败");
        return -1;
    }
    
    ROS_INFO("决策测试服务器: ROS节点已初始化，开始创建服务器对象...");
    
    try {
        DecisionTestServer server;
        ROS_INFO("决策测试服务器: 服务器对象创建完成，进入spin循环");
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("决策测试服务器启动失败: %s", e.what());
        return -1;
    }
    
    return 0;
}
