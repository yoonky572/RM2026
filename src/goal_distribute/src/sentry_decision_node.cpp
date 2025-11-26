/**
 * @file sentry_decision_node.cpp
 * @brief 哨兵决策主节点
 * 
 * 功能：
 * 1. 订阅游戏状态话题
 * 2. 根据游戏状态执行决策
 * 3. 发布目标点到导航系统
 * 
 * 注意：此节点用于实际运行，测试请使用 decision_test_server
 */

#include "goal_distribute/game_state.hpp"
#include "goal_distribute/goal_manager.hpp"
#include "goal_distribute/decision_engine.hpp"

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

/**
 * @brief 哨兵决策节点类
 * 
 * 负责：
 * - 接收游戏状态更新
 * - 执行决策逻辑
 * - 发布决策结果
 */
class SentryDecisionNode {
public:
    SentryDecisionNode() : nh_("~"), decision_rate_(10.0) {
        // 加载参数
        loadParameters();
        
        // 初始化目标点ID管理器（确保所有固定ID都存在）
        std::vector<int> goal_ids = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
        goal_manager_.addGoalIds(goal_ids);
        
        // 设置决策引擎默认策略
        decision_engine_.setDefaultStrategy(
            static_cast<goal_distribute::DecisionEngine::Strategy>(default_strategy_));
        
        // 加载JSON规则配置（如果指定了配置文件）
        std::string json_config_path;
        if (nh_.getParam("json_rules_file", json_config_path)) {
            if (decision_engine_.loadRulesFromJSON(json_config_path)) {
                decision_engine_.setUseJSONRules(true);
                ROS_INFO("Using JSON rules from: %s", json_config_path.c_str());
            } else {
                ROS_FATAL("Failed to load JSON rules from %s. Decision engine cannot operate without them.",
                          json_config_path.c_str());
            }
        } else {
            // 尝试使用默认路径
            std::string package_path = ros::package::getPath("goal_distribute");
            std::string default_json_path = package_path + "/config/decision_rules.json";
            if (decision_engine_.loadRulesFromJSON(default_json_path)) {
                decision_engine_.setUseJSONRules(true);
                ROS_INFO("Using default JSON rules from: %s", default_json_path.c_str());
            } else {
                ROS_FATAL("No JSON rules found at default path: %s. Please provide valid rule files.",
                          default_json_path.c_str());
            }
        }
        
        // 初始化发布器
        goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
        goal_id_pub_ = nh_.advertise<std_msgs::Int32>("/integer_topic", 10);
        strategy_pub_ = nh_.advertise<std_msgs::String>("/sentry_strategy", 10);
        
        // 统一状态订阅（推荐使用方式）
        game_state_sub_ = nh_.subscribe("/game_state/all", 10, 
                                       &SentryDecisionNode::gameStateCallback, this);
        
        // 或者，使用单独的话题订阅（备选方案）
        // hp_sub_ = nh_.subscribe("/game_state/self_hp", 10, 
        //                         &SentryDecisionNode::hpCallback, this);
        // ammo_sub_ = nh_.subscribe("/game_state/self_ammo", 10, 
        //                          &SentryDecisionNode::ammoCallback, this);
        // building_hp_sub_ = nh_.subscribe("/game_state/building_hp", 10, 
        //                                &SentryDecisionNode::buildingHpCallback, this);
        // enemy_sub_ = nh_.subscribe("/game_state/enemy", 10, 
        //                           &SentryDecisionNode::enemyCallback, this);
        
        // 注册状态更新回调
        game_state_.registerStateUpdateCallback(
            std::bind(&SentryDecisionNode::onStateUpdate, this));
        
        ROS_INFO("Sentry Decision Node initialized");
        ROS_INFO("Strategy: %d, Decision Rate: %.1f Hz", default_strategy_, decision_rate_);
        ROS_INFO("Waiting for game state messages on topic: /game_state/all");
    }
    
    void run() {
        ros::Rate rate(decision_rate_);
        
        while (ros::ok()) {
            // 执行决策
            makeAndPublishDecision();
            
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    void loadParameters() {
        nh_.param<double>("decision_rate", decision_rate_, 10.0);
        nh_.param<int>("default_strategy", default_strategy_, 2);  // 2 = BALANCED
        nh_.param<std::string>("team_color", team_color_, "red");
        
        // 初始化游戏状态
        game_state_.setTeamColor(team_color_);
        
        // 如果参数中有初始值，可以设置
        int initial_hp = 600;
        int initial_ammo = 500;
        nh_.param<int>("initial_hp", initial_hp, 600);
        nh_.param<int>("initial_ammo", initial_ammo, 500);
        game_state_.setSelfHP(initial_hp);
        game_state_.setSelfAmmo(initial_ammo);
    }
    
    void makeAndPublishDecision() {
        // 执行决策
        auto decision = decision_engine_.makeDecision(game_state_);
        
        if (decision.goal_id >= 0) {
            // 发布目标点ID（不再发布位置信息）
            std_msgs::Int32 goal_id_msg;
            goal_id_msg.data = decision.goal_id;
            goal_id_pub_.publish(goal_id_msg);
            
            // 发布策略信息
            std_msgs::String strategy_msg;
            strategy_msg.data = decision.reason;
            strategy_pub_.publish(strategy_msg);
            
            ROS_INFO_THROTTLE(1.0, "Decision: Goal ID=%d, Strategy=%d, Confidence=%.2f, Reason=%s",
                             decision.goal_id, 
                             static_cast<int>(decision.strategy),
                             decision.confidence,
                             decision.reason.c_str());
        } else {
            ROS_WARN_THROTTLE(1.0, "Failed to make decision: No available goals");
        }
    }
    
    void onStateUpdate() {
        // 状态更新时的回调，可以触发重新决策
        // 这里可以根据需要实现
    }
    
    // ===================================================================
    // TODO: 用户需要实现此函数 - 统一游戏状态回调
    // ===================================================================
    /**
     * @brief 统一游戏状态回调函数
     * 
     * 此函数接收所有游戏状态信息，需要用户根据实际的消息格式实现解析逻辑
     * 
     * 需要从消息中提取的信息：
     * 1. 自身血量 (self_hp)
     * 2. 自身弹丸数量 (self_ammo)
     * 3. 友方建筑血量：
     *    - 基地血量 (BASE)
     *    - 前哨站血量 (SENTRY_OUTPOST)
     * 4. 敌方建筑血量（同上）
     * 5. 敌人状态：
     *    - 是否可见 (enemy_visible)
     *    - 敌人位置 (enemy_x, enemy_y, enemy_z)
     * 6. 比赛时间（可选）
     * 
     * 实现示例：
     * 
     * void SentryDecisionNode::gameStateCallback(const YourGameStateMsg::ConstPtr& msg) {
     *     // 1. 解析自身状态
     *     game_state_.setSelfHP(msg->self_hp);
     *     game_state_.setSelfAmmo(msg->self_ammo);
     *     
     *     // 2. 解析建筑血量
     *     std::map<GameState::BuildingType, uint16_t> ally_buildings;
     *     std::map<GameState::BuildingType, uint16_t> enemy_buildings;
     *     
     *     ally_buildings[GameState::BuildingType::BASE] = msg->ally_base_hp;
     *     ally_buildings[GameState::BuildingType::SENTRY_OUTPOST] = msg->ally_outpost_hp;
     *     
     *     enemy_buildings[GameState::BuildingType::BASE] = msg->enemy_base_hp;
     *     enemy_buildings[GameState::BuildingType::SENTRY_OUTPOST] = msg->enemy_outpost_hp;
     *     
     *     // 3. 解析敌人状态
     *     bool enemy_visible = msg->enemy_detected;
     *     double enemy_x = msg->enemy_pose.position.x;
     *     double enemy_y = msg->enemy_pose.position.y;
     *     double enemy_z = msg->enemy_pose.position.z;
     *     
     *     // 4. 批量更新状态（推荐使用此方法）
     *     game_state_.updateAllState(
     *         msg->self_hp, 
     *         msg->self_ammo,
     *         ally_buildings,
     *         enemy_buildings,
     *         enemy_visible,
     *         enemy_x, enemy_y, enemy_z
     *     );
     *     
     *     // 或者分别更新：
     *     // game_state_.setSelfHP(msg->self_hp);
     *     // game_state_.setSelfAmmo(msg->self_ammo);
     *     // game_state_.setAllBuildingHP(ally_buildings, enemy_buildings);
     *     // if (enemy_visible) {
     *     //     game_state_.setEnemyPosition(enemy_x, enemy_y, enemy_z);
     *     // }
     * }
     * 
     * 注意：
     * - 根据实际的消息类型修改函数参数类型
     * - 根据实际的消息字段名称调整解析逻辑
     * - 确保所有字段都正确映射
     * 
     * ===================================================================
     */
    void gameStateCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // ===================================================================
        // TODO: 用户需要根据实际消息类型实现此函数
        // ===================================================================
        // 
        // 当前使用 geometry_msgs::PoseStamped 作为占位符
        // 用户需要：
        // 1. 根据实际的消息类型修改函数参数
        // 2. 实现消息解析逻辑
        // 3. 调用 game_state_.updateAllState() 或分别调用各个 set 方法
        //
        // 参考上面的注释中的示例代码
        //
        // ===================================================================
        
        ROS_WARN_THROTTLE(5.0, "gameStateCallback() is not implemented yet. "
                               "Please implement this function to parse game state messages.");
        ROS_WARN_THROTTLE(5.0, "Current message type is Placeholder. "
                               "Please modify to use the actual message type from your game system.");
        
        // 临时实现示例（仅用于测试）
        // 用户需要删除此代码并实现真正的解析逻辑
        // game_state_.setSelfHP(600);
        // game_state_.setSelfAmmo(500);
    }
    
    // ===================================================================
    // 以下函数为备选方案，如果使用单独话题订阅状态，可以实现这些函数
    // ===================================================================
    
    void hpCallback(const std_msgs::Int32::ConstPtr& msg) {
        game_state_.setSelfHP(static_cast<uint16_t>(msg->data));
    }
    
    void ammoCallback(const std_msgs::Int32::ConstPtr& msg) {
        game_state_.setSelfAmmo(static_cast<uint16_t>(msg->data));
    }
    
    // ===================================================================
    // TODO: 用户需要实现此函数 - 建筑血量回调
    // ===================================================================
    /**
     * @brief 建筑血量回调函数
     * 
     * 如果使用单独的话题订阅建筑血量，需要实现此函数
     * 
     * 需要从消息中提取：
     * - 建筑类型（BASE, SENTRY_OUTPOST）
     * - 是否为友方建筑
     * - 血量值
     * 
     * 然后调用：
     * game_state_.setBuildingHP(building_type, hp, is_ally);
     * 
     * 或者批量更新：
     * game_state_.setAllBuildingHP(ally_buildings, enemy_buildings);
     */
    void buildingHpCallback(const std_msgs::Int32::ConstPtr& msg) {
        // ===================================================================
        // TODO: 用户需要根据实际消息格式实现此函数
        // ===================================================================
        // 
        // 如果使用自定义消息类型，修改函数参数类型
        // 然后解析建筑类型、是否友方、血量值
        // 
        // 示例：
        // BuildingHPMsg::ConstPtr building_msg = ...;
        // GameState::BuildingType type = parseBuildingType(building_msg->type);
        // bool is_ally = building_msg->is_ally;
        // uint16_t hp = building_msg->hp;
        // game_state_.setBuildingHP(type, hp, is_ally);
        //
        // ===================================================================
        
        ROS_WARN_THROTTLE(5.0, "buildingHpCallback() is not implemented yet. "
                               "Please implement this function to parse building HP messages.");
        
        // 简化示例：假设消息格式为 building_type * 1000 + hp
        // 实际应该使用自定义消息类型
        // int building_type_id = msg->data / 1000;
        // int hp = msg->data % 1000;
        // GameState::BuildingType type = static_cast<GameState::BuildingType>(building_type_id);
        // game_state_.setBuildingHP(type, hp, true);
    }
    
    void enemyCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // 敌人可见
        game_state_.setEnemyPosition(msg->pose.position.x,
                                    msg->pose.position.y,
                                    msg->pose.position.z);
    }
    
    ros::NodeHandle nh_;
    double decision_rate_;
    int default_strategy_;
    std::string team_color_;
    
    goal_distribute::GameState game_state_;
    goal_distribute::GoalManager goal_manager_;
    goal_distribute::DecisionEngine decision_engine_;
    
    ros::Publisher goal_pub_;
    ros::Publisher goal_id_pub_;
    ros::Publisher strategy_pub_;
    
    // 统一状态订阅器（推荐）
    ros::Subscriber game_state_sub_;
    
    // 单独状态订阅器（备选方案，如果不需要可以注释掉）
    ros::Subscriber hp_sub_;
    ros::Subscriber ammo_sub_;
    ros::Subscriber building_hp_sub_;
    ros::Subscriber enemy_sub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "sentry_decision_node");
    
    try {
        SentryDecisionNode node;
        node.run();
    } catch (const std::exception& e) {
        ROS_ERROR("Sentry Decision Node error: %s", e.what());
        return -1;
    }
    
    return 0;
}