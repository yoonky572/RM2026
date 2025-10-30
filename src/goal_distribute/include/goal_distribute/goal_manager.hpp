#ifndef GOAL_DISTRIBUTE_GOAL_MANAGER_HPP
#define GOAL_DISTRIBUTE_GOAL_MANAGER_HPP

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <vector>
#include <string>
#include <map>
#include <mutex>

namespace goal_distribute {

/**
 * @brief 目标点管理器
 * 管理所有可用的目标点位置，支持动态配置
 */
class GoalManager {
public:
    struct GoalPoint {
        geometry_msgs::PoseStamped pose;
        std::string name;
        int priority;  // 优先级，数值越大优先级越高
        bool enabled;  // 是否启用
    };

    GoalManager();
    ~GoalManager() = default;

    // ========== 目标点管理 ==========
    bool addGoalPoint(int id, const geometry_msgs::PoseStamped& pose, 
                     const std::string& name = "", int priority = 0);
    bool removeGoalPoint(int id);
    bool updateGoalPoint(int id, const geometry_msgs::PoseStamped& pose);
    bool setGoalEnabled(int id, bool enabled);
    bool setGoalPriority(int id, int priority);

    // ========== 目标点查询 ==========
    bool getGoalPoint(int id, geometry_msgs::PoseStamped& pose) const;
    std::vector<int> getAvailableGoals() const;
    std::vector<int> getGoalsByPriority(int min_priority = 0) const;

    // ========== ROS参数加载 ==========
    bool loadGoalsFromROSParam(const ros::NodeHandle& nh, const std::string& team_color);

    // ========== 按场地位置设定目标点 ==========
    /**
     * @brief 根据RoboMaster场地位置设定目标点
     * 这个方法应该根据实际的场地布局来设置目标点
     * 
     * TODO: 用户需要根据实际场地布局实现此函数
     * 示例场地位置：
     * - 基地附近防御点
     * - 前哨站附近防御点
     * - 中场控制点
     * - 攻击位置
     * - 撤退安全点
     * 等等
     * 
     * @param team_color 队伍颜色 "red" 或 "blue"
     * @return 是否成功设置
     */
    bool initializeFieldGoals(const std::string& team_color);
    
    /**
     * @brief 添加场地特定位置的目标点
     * 这是一个辅助函数，用于方便地添加已知的场地位置
     * 
     * @param field_position_name 场地位置名称（如 "base_defense", "outpost_near", 等）
     * @param pose 目标点的位姿
     * @param priority 优先级
     * @return 返回分配的目标点ID，失败返回-1
     */
    int addFieldGoal(const std::string& field_position_name, 
                    const geometry_msgs::PoseStamped& pose, 
                    int priority = 0);

    // ========== 获取目标点数量 ==========
    size_t getGoalCount() const;

private:
    mutable std::mutex goals_mutex_;
    std::map<int, GoalPoint> goals_;
};

} // namespace goal_distribute

#endif // GOAL_DISTRIBUTE_GOAL_MANAGER_HPP
