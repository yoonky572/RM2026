#pragma once

#include <ros/ros.h>
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>

namespace recovery {

/**
 * @brief FindFreespace 恢复行为：当机器人被困在障碍物中时，寻找自由空间并发布速度命令逃离
 * 
 * 功能说明：
 * 1. 获取机器人当前位置
 * 2. 检查机器人周围是否有足够的自由空间
 * 3. 如果被困，在搜索半径内寻找自由空间
 * 4. 计算障碍物中心，选择距离障碍物中心最远的自由空间点
 * 5. 计算并发布速度命令，使机器人向自由空间移动
 * 6. 发布可视化标记
 */
class FindFreespace : public nav_core::RecoveryBehavior {
public:
    FindFreespace();
    ~FindFreespace();

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
     * @brief 执行恢复行为：检查机器人是否被困，如果是则发布速度命令逃离
     */
    void runBehavior();

private:
    /**
     * @brief 检查机器人周围是否有足够的自由空间
     * @param costmap 代价地图指针
     * @param grid_x 机器人栅格坐标 x
     * @param grid_y 机器人栅格坐标 y
     * @return 是否有足够的自由空间
     */
    bool isSurroundedByFreeSpace(const costmap_2d::Costmap2D* costmap, 
                                  int grid_x, int grid_y) const;

    /**
     * @brief 计算并发布速度命令
     * @param robot_pose 机器人位姿
     * @param target_position 目标位置
     * @param duration 发布持续时间（秒）
     */
    void publishVelocityCommand(const geometry_msgs::PoseStamped& robot_pose,
                                const geometry_msgs::Point& target_position,
                                double duration);

    /**
     * @brief 发布可视化标记
     * @param position 标记位置
     * @param frame_id 坐标系
     */
    void publishMarker(const geometry_msgs::Point& position, const std::string& frame_id);

    // ROS 相关
    ros::NodeHandle nh_;                    // 节点句柄
    ros::Publisher cmd_pub_;                // 速度命令发布器
    ros::Publisher marker_pub_;             // 可视化标记发布器

    // 代价地图
    costmap_2d::Costmap2DROS* global_costmap_;  // 全局代价地图
    costmap_2d::Costmap2DROS* local_costmap_;   // 局部代价地图（未使用，保留接口兼容性）

    // 参数
    std::string name_;                      // 行为名称
    double max_search_radius_;              // 最大搜索半径（米）
    double escape_speed_;                   // 逃离速度（米/秒）
    double command_duration_;                // 命令发布持续时间（秒）
};

} // namespace recovery
