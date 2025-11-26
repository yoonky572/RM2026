#pragma once

#include <ros/ros.h>
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/buffer.h>

namespace recovery {

/**
 * @brief Goal_Reset 恢复行为：当目标点位于障碍物中时，自动寻找最近的自由空间并重置目标点
 * 
 * 功能说明：
 * 1. 监听目标点话题，保存初始目标
 * 2. 当目标点在障碍物中时，在搜索半径内寻找自由空间
 * 3. 计算障碍物中心，选择距离障碍物中心最远的自由空间点作为新目标
 * 4. 发布新的目标点和可视化标记
 */
class Goal_Reset : public nav_core::RecoveryBehavior {
public:
    Goal_Reset();
    ~Goal_Reset();

    /**
     * @brief 初始化恢复行为
     * @param name 行为名称
     * @param tf TF 缓冲区指针
     * @param global_costmap 全局代价地图
     * @param local_costmap 局部代价地图
     */
    void initialize(std::string name, 
                    tf2_ros::Buffer* tf, 
                    costmap_2d::Costmap2DROS* global_costmap,
                    costmap_2d::Costmap2DROS* local_costmap);

    /**
     * @brief 执行恢复行为：检查目标点是否在障碍物中，如果是则重置目标点
     */
    void runBehavior();

private:
    /**
     * @brief 目标点回调函数，保存初始目标点
     * @param msg 目标点消息
     */
    void goalCallback(const geometry_msgs::PoseStampedConstPtr& msg);

    /**
     * @brief 发布可视化标记
     * @param position 标记位置
     * @param frame_id 坐标系
     */
    void publishMarker(const geometry_msgs::Point& position, const std::string& frame_id);

    // ROS 相关
    ros::NodeHandle nh_;                    // 节点句柄
    ros::Publisher goal_pub_;               // 目标点发布器
    ros::Publisher marker_pub_;             // 可视化标记发布器
    ros::Subscriber goal_sub_;              // 目标点订阅器

    // 代价地图
    costmap_2d::Costmap2DROS* global_costmap_;  // 全局代价地图
    costmap_2d::Costmap2DROS* local_costmap_;   // 局部代价地图（未使用，保留接口兼容性）

    // 参数
    std::string name_;                      // 行为名称
    double max_search_radius_;              // 最大搜索半径（米）

    // 状态
    geometry_msgs::PoseStamped initial_goal_;  // 初始目标点
};

} // namespace recovery
