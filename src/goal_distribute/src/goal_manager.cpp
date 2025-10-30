#include "goal_distribute/goal_manager.hpp"
#include <ros/ros.h>
#include <algorithm>

namespace goal_distribute {

GoalManager::GoalManager() {
}

bool GoalManager::addGoalPoint(int id, const geometry_msgs::PoseStamped& pose, 
                               const std::string& name, int priority) {
    std::lock_guard<std::mutex> lock(goals_mutex_);
    GoalPoint goal;
    goal.pose = pose;
    goal.name = name.empty() ? "goal_" + std::to_string(id) : name;
    goal.priority = priority;
    goal.enabled = true;
    
    goals_[id] = goal;
    return true;
}

bool GoalManager::removeGoalPoint(int id) {
    std::lock_guard<std::mutex> lock(goals_mutex_);
    return goals_.erase(id) > 0;
}

bool GoalManager::updateGoalPoint(int id, const geometry_msgs::PoseStamped& pose) {
    std::lock_guard<std::mutex> lock(goals_mutex_);
    auto it = goals_.find(id);
    if (it != goals_.end()) {
        it->second.pose = pose;
        return true;
    }
    return false;
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

bool GoalManager::getGoalPoint(int id, geometry_msgs::PoseStamped& pose) const {
    std::lock_guard<std::mutex> lock(goals_mutex_);
    auto it = goals_.find(id);
    if (it != goals_.end() && it->second.enabled) {
        pose = it->second.pose;
        return true;
    }
    return false;
}

std::vector<int> GoalManager::getAvailableGoals() const {
    std::lock_guard<std::mutex> lock(goals_mutex_);
    std::vector<int> available;
    for (const auto& pair : goals_) {
        if (pair.second.enabled) {
            available.push_back(pair.first);
        }
    }
    return available;
}

std::vector<int> GoalManager::getGoalsByPriority(int min_priority) const {
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

bool GoalManager::loadGoalsFromROSParam(const ros::NodeHandle& nh, const std::string& team_color) {
    std::lock_guard<std::mutex> lock(goals_mutex_);
    
    // 获取目标点数量
    int goal_num = 0;
    if (!nh.getParam("goal_num", goal_num) || goal_num <= 0) {
        ROS_WARN("Failed to get goal_num parameter, using default value 11");
        goal_num = 11;
    }
    
    bool success = true;
    for (int i = 0; i < goal_num; ++i) {
        std::string param_name = team_color + "_goal" + std::to_string(i);
        std::vector<double> goal_vec;
        
        if (nh.getParam(param_name, goal_vec)) {
            if (goal_vec.size() == 7) {
                geometry_msgs::PoseStamped pose;
                pose.header.frame_id = "map";
                pose.header.stamp = ros::Time::now();
                pose.pose.position.x = goal_vec[0];
                pose.pose.position.y = goal_vec[1];
                pose.pose.position.z = goal_vec[2];
                pose.pose.orientation.x = goal_vec[3];
                pose.pose.orientation.y = goal_vec[4];
                pose.pose.orientation.z = goal_vec[5];
                pose.pose.orientation.w = goal_vec[6];
                
                addGoalPoint(i, pose, param_name, 0);
            } else {
                ROS_WARN("Goal point %s has invalid size: %zu (expected 7)", 
                         param_name.c_str(), goal_vec.size());
                success = false;
            }
        } else {
            ROS_WARN("Failed to load goal point: %s", param_name.c_str());
            success = false;
        }
    }
    
    ROS_INFO("Loaded %zu goal points from ROS parameters", goals_.size());
    return success;
}

bool GoalManager::initializeFieldGoals(const std::string& team_color) {
    std::lock_guard<std::mutex> lock(goals_mutex_);
    
    // ===================================================================
    // TODO: 用户需要根据实际RoboMaster场地布局实现此函数
    // ===================================================================
    // 
    // 此函数应该根据场地实际位置设置目标点
    // 
    // 建议的目标点类型：
    // 1. 基地防御点（高优先级，priority=10）
    // 2. 前哨站防御点（高优先级，priority=9）
    // 3. 中场控制点（中优先级，priority=5）
    // 4. 攻击位置（中优先级，priority=4）
    // 5. 侧翼位置（低优先级，priority=2）
    // 6. 撤退安全点（高优先级，priority=8）
    // 
    // 示例代码结构：
    // 
    // int goal_id = 0;
    // 
    // // 基地防御点
    // geometry_msgs::PoseStamped base_defense;
    // base_defense.header.frame_id = "map";
    // base_defense.pose.position.x = [根据场地设置的x坐标];
    // base_defense.pose.position.y = [根据场地设置的y坐标];
    // base_defense.pose.position.z = 0.0;
    // base_defense.pose.orientation = [根据场地设置的朝向];
    // addGoalPoint(goal_id++, base_defense, "base_defense_0", 10);
    // 
    // // 前哨站防御点
    // geometry_msgs::PoseStamped outpost_defense;
    // [类似的设置...]
    // addGoalPoint(goal_id++, outpost_defense, "outpost_defense_0", 9);
    // 
    // // 以此类推...
    // 
    // ===================================================================
    
    ROS_WARN("initializeFieldGoals() is not implemented yet. "
             "Please implement this function according to the actual field layout.");
    ROS_WARN("You can use addFieldGoal() to add individual field goals, "
             "or implement initializeFieldGoals() to set all goals at once.");
    
    // 临时实现：返回false表示未实现
    // 用户可以：
    // 1. 直接在launch文件中配置目标点（通过loadGoalsFromROSParam）
    // 2. 或者实现此函数来根据场地布局设置目标点
    return false;
}

int GoalManager::addFieldGoal(const std::string& field_position_name, 
                             const geometry_msgs::PoseStamped& pose, 
                             int priority) {
    std::lock_guard<std::mutex> lock(goals_mutex_);
    
    // 查找下一个可用的ID
    int next_id = 0;
    if (!goals_.empty()) {
        next_id = goals_.rbegin()->first + 1;
    }
    
    GoalPoint goal;
    goal.pose = pose;
    goal.name = field_position_name;
    goal.priority = priority;
    goal.enabled = true;
    
    goals_[next_id] = goal;
    ROS_INFO("Added field goal: ID=%d, Name=%s, Priority=%d", 
             next_id, field_position_name.c_str(), priority);
    
    return next_id;
}

size_t GoalManager::getGoalCount() const {
    std::lock_guard<std::mutex> lock(goals_mutex_);
    return goals_.size();
}

} // namespace goal_distribute
