#include "goal_distribute/goal_manager.hpp"
#include <ros/ros.h>
#include <algorithm>

namespace goal_distribute {

GoalManager::GoalManager() {
}

bool GoalManager::addGoal(int id, const std::string& name, int priority) {
    std::lock_guard<std::mutex> lock(goals_mutex_);
    GoalInfo info;
    info.id = id;
    info.name = name.empty() ? "goal_" + std::to_string(id) : name;
    info.priority = priority;
    info.enabled = true;
    goals_[id] = info;
    return true;
}

bool GoalManager::removeGoal(int id) {
    std::lock_guard<std::mutex> lock(goals_mutex_);
    return goals_.erase(id) > 0;
}

bool GoalManager::setGoalEnabled(int id, bool enabled) {
    std::lock_guard<std::mutex> lock(goals_mutex_);
    auto it = goals_.find(id);
    if (it != goals_.end()) {
        it->second.enabled = enabled;
        return true;
    }
    return false;
}

bool GoalManager::setGoalPriority(int id, int priority) {
    std::lock_guard<std::mutex> lock(goals_mutex_);
    auto it = goals_.find(id);
    if (it != goals_.end()) {
        it->second.priority = priority;
        return true;
    }
    return false;
}

bool GoalManager::isGoalEnabled(int id) const {
    std::lock_guard<std::mutex> lock(goals_mutex_);
    auto it = goals_.find(id);
    return it != goals_.end() && it->second.enabled;
}

std::vector<int> GoalManager::getAvailableGoalIds() const {
    std::lock_guard<std::mutex> lock(goals_mutex_);
    std::vector<int> available;
    for (const auto& pair : goals_) {
        if (pair.second.enabled) {
            available.push_back(pair.first);
        }
    }
    return available;
}

std::vector<int> GoalManager::getGoalIdsByPriority(int min_priority) const {
    std::lock_guard<std::mutex> lock(goals_mutex_);
    std::vector<std::pair<int, int>> goal_priorities;  // (goal_id, priority)
    
    for (const auto& pair : goals_) {
        if (pair.second.enabled && pair.second.priority >= min_priority) {
            goal_priorities.push_back({pair.first, pair.second.priority});
        }
    }
    
    // 按优先级排序
    std::sort(goal_priorities.begin(), goal_priorities.end(),
              [](const std::pair<int, int>& a, const std::pair<int, int>& b) {
                  return a.second > b.second;
              });
    
    std::vector<int> result;
    for (const auto& pair : goal_priorities) {
        result.push_back(pair.first);
    }
    return result;
}

bool GoalManager::addGoalIds(const std::vector<int>& goal_ids) {
    std::lock_guard<std::mutex> lock(goals_mutex_);
    for (int id : goal_ids) {
        GoalInfo info;
        info.id = id;
        info.name = "goal_" + std::to_string(id);
        info.priority = 0;
        info.enabled = true;
        goals_[id] = info;
    }
    ROS_INFO("Added %zu goal IDs", goal_ids.size());
    return true;
}

size_t GoalManager::getGoalCount() const {
    std::lock_guard<std::mutex> lock(goals_mutex_);
    return goals_.size();
}

} // namespace goal_distribute
