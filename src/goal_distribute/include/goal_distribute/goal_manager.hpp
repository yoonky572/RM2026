#ifndef GOAL_DISTRIBUTE_GOAL_MANAGER_HPP
#define GOAL_DISTRIBUTE_GOAL_MANAGER_HPP

#include <vector>
#include <string>
#include <map>
#include <mutex>

namespace goal_distribute {

/**
 * @brief 目标点ID管理器（简化版，不管理位置信息）
 * 只负责维护目标点ID的启用状态和优先级
 */
class GoalManager {
public:
    struct GoalInfo {
        int id;
        std::string name;
        int priority;  // 优先级，数值越大优先级越高
        bool enabled;  // 是否启用
    };

    GoalManager();
    ~GoalManager() = default;

    // ========== 目标点ID管理 ==========
    bool addGoal(int id, const std::string& name = "", int priority = 0);
    bool removeGoal(int id);
    bool setGoalEnabled(int id, bool enabled);
    bool setGoalPriority(int id, int priority);
    bool isGoalEnabled(int id) const;

    // ========== 目标点ID查询 ==========
    std::vector<int> getAvailableGoalIds() const;
    std::vector<int> getGoalIdsByPriority(int min_priority = 0) const;

    // ========== 批量操作 ==========
    /**
     * @brief 批量添加目标点ID（用于固定策略ID映射）
     * @param goal_ids 目标点ID列表
     * @return 是否成功
     */
    bool addGoalIds(const std::vector<int>& goal_ids);

    // ========== 获取目标点数量 ==========
    size_t getGoalCount() const;

private:
    mutable std::mutex goals_mutex_;
    std::map<int, GoalInfo> goals_;
};

} // namespace goal_distribute

#endif // GOAL_DISTRIBUTE_GOAL_MANAGER_HPP
