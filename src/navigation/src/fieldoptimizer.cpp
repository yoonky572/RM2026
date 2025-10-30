/**
 * @file fieldoptimizer.cpp
 * @brief 势场法路径优化器实现
 * 
 * 使用人工势场法优化路径点，使路径远离障碍物。
 * 通过计算障碍物的斥力和目标点的引力，迭代优化路径点位置。
 */

#include "navigation/fieldoptimizer.hpp"

/**
 * @brief 设置优化器参数
 * @param param 优化器参数结构体
 */
void Field_Optimizer::setparam(const Params& param)
{
    param_ = param;
}

/**
 * @brief 计算障碍物对路径点的斥力
 * @param pose 当前路径点位置
 * @param obstacle 障碍物信息
 * @return 斥力向量
 * 
 * 斥力大小与距离的倒数相关，距离越近斥力越大
 */
Eigen::Vector2d Field_Optimizer::force_calculate(const geometry_msgs::PoseStamped& pose,
                                                  const ObstacleResult& obstacle)
{
    Eigen::Vector2d force(0, 0);
    
    // 计算从障碍物指向路径点的向量
    Eigen::Vector2d delta(pose.pose.position.x - obstacle.world_position.x,
                          pose.pose.position.y - obstacle.world_position.y);
    double distance = delta.norm();
    
    // 如果距离超过安全距离，斥力为0
    if (distance > param_.safe_distance)
    {
        return force;
    }

    // 防止除以零，设置最小距离
    const double EPSILON = 1e-3;
    distance = std::max(distance, EPSILON);
    
    // 计算斥力：F_rep = k_rep * (1/d - 1/d_safe)^2 * direction
    double term = 1.0 / distance - 1.0 / param_.safe_distance;
    double force_magnitude = param_.repulsive_gain * pow(term, 2);
    force = delta.normalized() * force_magnitude;
    
    return force;
}

/**
 * @brief 计算目标点对路径点的引力
 * @param tar_pose 目标路径点位置
 * @param cur_pose 当前路径点位置
 * @return 引力向量
 * 
 * 引力大小与距离的平方成正比，保证路径点向目标点靠拢
 */
Eigen::Vector2d Field_Optimizer::attractive_calculate(
    const geometry_msgs::PoseStamped& tar_pose,
    const geometry_msgs::PoseStamped& cur_pose)
{
    Eigen::Vector2d force;
    
    // 计算从当前点指向目标点的向量
    Eigen::Vector2d delta(tar_pose.pose.position.x - cur_pose.pose.position.x,
                          tar_pose.pose.position.y - cur_pose.pose.position.y);
    double distance = delta.norm();
    
    // 计算引力：F_att = k_att * d^2 * direction
    double force_magnitude = param_.attractive_gain * pow(distance, 2);
    force = delta.normalized() * force_magnitude;
    
    return force;
}

/**
 * @brief 优化单个路径点位置
 * @param result 障碍物检测结果列表
 * @param pose 原始路径点位置
 * @param pose_opt 优化后的路径点位置（输出）
 * @param ratio 优化强度比例（0-1），控制优化幅度
 * @return 优化是否成功
 * 
 * 使用梯度下降法迭代优化路径点，使其在远离障碍物的同时尽量接近原始位置
 */
bool Field_Optimizer::optimize(const std::vector<ObstacleResult>& result,
                                const geometry_msgs::PoseStamped& pose,
                                geometry_msgs::PoseStamped& pose_opt,
                                double ratio)
{
    int iteration = 0;
    pose_opt = pose;  // 初始化为原始位置
    
    while (iteration < param_.max_iterations)
    {
        Eigen::Vector2d total_force(0, 0);
        
        // ========== 计算所有障碍物的斥力总和 ==========
        for (const auto& obs : result)
        {
            total_force += force_calculate(pose_opt, obs);
        }
        
        // ========== 计算目标点的引力 ==========
        // 确保优化后的点不会偏离原始位置太远
        total_force += attractive_calculate(pose, pose_opt);
        
        // ========== 计算步长并更新位置 ==========
        double step_size = std::min(param_.max_step, total_force.norm());
        
        if (total_force.norm() > 1e-6)  // 避免归一化零向量
        {
            Eigen::Vector2d step = total_force.normalized() * step_size;
            pose_opt.pose.position.x += step.x();
            pose_opt.pose.position.y += step.y();
        }
        
        // ========== 检查收敛条件 ==========
        if (step_size < param_.convergence_thresh)
        {
            // 根据ratio调整优化幅度，避免过度优化
            double dx = pose_opt.pose.position.x - pose.pose.position.x;
            double dy = pose_opt.pose.position.y - pose.pose.position.y;
            pose_opt.pose.position.x = dx * ratio + pose.pose.position.x;
            pose_opt.pose.position.y = dy * ratio + pose.pose.position.y;
            
            return true;  // 优化成功
        }
        
        iteration++;
    }
    
    // 达到最大迭代次数仍未收敛
    return false;
}

/**
 * @brief 全局路径回调函数
 * @param msg 接收到的全局路径消息
 */
void Field_Optimizer::GlobalPathCallback(const nav_msgs::PathConstPtr& msg)
{
    global_path = *msg;
}
