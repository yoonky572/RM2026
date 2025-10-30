#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <mutex>
#include <cmath>
#include <chrono>
#include <iostream>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2/utils.h>
#include <tf/transform_listener.h>
#include <Eigen/Eigen>

#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GridCells.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>

#include "utility.hpp"
#include "navigation/fieldoptimizer.hpp"

/**
 * @brief 导航控制器类
 * 
 * 负责路径跟踪、障碍物避障、路径优化和速度控制
 */
class Controller
{
public:
    Controller();
    ~Controller();

    // ========== ROS回调函数（公共接口） ==========
    void GlobalPathCallback(const nav_msgs::PathConstPtr& msg);
    void CostmapCallback(const nav_msgs::OccupancyGridConstPtr& msg);
    void MatchCallback(const std_msgs::BoolConstPtr& msg);
    void LocalizeCallback(const std_msgs::BoolConstPtr& msg);
    void StatusCallback(const actionlib_msgs::GoalStatusArray& msg);

    // ========== 主要控制循环 ==========
    void Plan(const ros::TimerEvent& event);
    void PathOptimaze(const ros::TimerEvent& event);

private:
    // ========== 参数加载和初始化辅助函数 ==========
    void loadParameters(ros::NodeHandle& nh);
    void initializePublishers(ros::NodeHandle& nh);
    void initializeSubscribers(ros::NodeHandle& nh);
    void initializeTimers(ros::NodeHandle& nh);

    // ========== 状态检查和判断函数 ==========
    bool isGoalReached(const geometry_msgs::PoseStamped& robot_pose, double distance_to_goal) const;
    bool isLocalizationReady() const;
    bool canExecutePath() const;
    bool shouldTurnInPlace(double yaw_error, double distance_to_goal) const;
    bool isTurnComplete(double yaw_error) const;

    // ========== 路径处理函数 ==========
    nav_msgs::Path extractPrunePathFromGlobal(const geometry_msgs::PoseStamped& robot_pose);
    std::vector<geometry_msgs::PoseStamped> optimizePathPoints(
        const std::vector<geometry_msgs::PoseStamped>& prune_path, 
        const geometry_msgs::PoseStamped& robot_pose);
    void removeClosePathPoints(std::vector<geometry_msgs::PoseStamped>& path, double min_distance);
    nav_msgs::Path extractFollowingPath(const geometry_msgs::PoseStamped& robot_pose);

    // ========== 障碍物检测函数 ==========
    void updateNarrowPassageStatus(int obstacle_count);
    void detectNarrowPassage(const nav_msgs::Path& path);

    // ========== 速度控制计算函数 ==========
    void handleGoalReached();
    void handleWaitingForLocalization();
    void handleTurningInPlace(double yaw_error);
    void handlePathTracking(const geometry_msgs::PoseStamped& robot_pose, 
                            const nav_msgs::Path& path, 
                            double distance_to_goal);
    void applySpeedScalingNearGoal(geometry_msgs::Twist& cmd_vel, double distance_to_goal);
    void publishStopCommand(int status_flag);

    // ========== 路径工具函数 ==========
    void FindNearstPose(geometry_msgs::PoseStamped& robot_pose,
                       nav_msgs::Path& path,
                       int& prune_index,
                       double prune_ahead_dist);
    geometry_msgs::PoseStamped FindNearstFreeSpace(
        geometry_msgs::PoseStamped& robot_pose,
        nav_msgs::OccupancyGrid& costmap,
        double min_radius,
        double max_radius);
    int Passbility_check(nav_msgs::OccupancyGrid& costmap,
                         nav_msgs::Path& plan,
                         double search_radius,
                         std::vector<ObstacleResult>& result,
                         int forsee_index);
    void publishObstaclesGrid(const std::vector<ObstacleResult>& obstacles,
                              double resolution,
                              ros::Publisher& pub);

    // ========== 控制计算函数 ==========
    void FollowTraj(const geometry_msgs::PoseStamped& robot_pose,
                    const nav_msgs::Path& traj,
                    geometry_msgs::Twist& cmd_vel);
    double YawErrorCal(const geometry_msgs::PoseStamped& robot_pose,
                       const geometry_msgs::PoseStamped& path_pose,
                       bool x_forward);
    double CurvatureCal(const nav_msgs::Path& traj);
    double YawControl(const geometry_msgs::PoseStamped& robot_pose,
                      const geometry_msgs::PoseStamped& path_pose,
                      bool x_forward);
    double YawControl(double error);
    bool determineForwardDirection() const;

    // ========== ROS接口成员 ==========
    ros::Publisher cmd_vel_pub;
    ros::Publisher prune_path_pub;
    ros::Publisher local_path_pub;
    ros::Publisher forsee_path_pub;
    ros::Publisher obstacle_pub;
    ros::Publisher narrow_pub;
    ros::Subscriber diverge_sub;
    ros::Subscriber match_sub;
    ros::Subscriber localize_success_sub;
    ros::Subscriber global_path_sub;
    ros::Subscriber costmap_sub;
    ros::Subscriber global_status_sub;
    ros::Timer plan_timer;
    ros::Timer optimize_timer;

    // ========== TF和状态管理 ==========
    std::shared_ptr<tf::TransformListener> tf_listener;
    std::chrono::high_resolution_clock::time_point localized_time;

    // ========== 路径数据 ==========
    nav_msgs::Path global_path;         // 全局规划路径
    nav_msgs::Path prune_path;          // 剪枝后的可通行路径
    nav_msgs::Path opt_path;            // 优化后的局部路径
    nav_msgs::OccupancyGrid costmap;    // 代价地图
    std::vector<ObstacleResult> obstacle_result;  // 障碍物检测结果
    
    // ========== 路径索引和状态 ==========
    int prune_index;                    // 全局路径剪枝索引
    int follow_index;                   // 跟随路径索引
    
    // ========== 系统状态标志 ==========
    bool plan;                          // 是否有有效规划
    bool localized;                     // 是否定位成功
    bool diverge;                       // 是否定位发散
    bool turn_state;                    // 是否处于转向状态
    bool arrive_state;                  // 是否到达目标
    bool narrow;                        // 是否处于狭窄通道
    bool x_forward;                     // 前进方向标志
    int global_planner_status;          // 全局规划器状态

    // ========== 控制参数 ==========
    // 速度参数
    double max_speed;
    double set_yaw_speed;
    double straight_p_value;
    double curve_p_value;
    double p_value;
    
    // 航向控制参数
    double wz_p_value;
    double wz_d_value;
    double last_yaw_error;
    
    // 路径规划参数
    int plan_freq;
    int opt_freq;
    double goal_dist_tolerance;
    double prune_ahead_dist;
    
    // 前视距离参数
    int straight_foresee_index;
    int curve_foresee_index;
    int forsee_index;
    int short_forsee_index;
    
    // 障碍物检测参数
    int narrow_threshold;
    double search_radius;
    
    // 恢复行为参数
    double min_recover_radius;
    double max_recover_radius;
    
    // 调试和模式参数
    bool debug_en;
    bool hole_mode;
    std::string global_frame;
    
    // ========== 计算中间变量 ==========
    double curvature;
    double yaw;
    std_msgs::Bool narrow_msg;
    
    // ========== 路径优化器 ==========
    Field_Optimizer optimizer_;
    Field_Optimizer::Params param_;
    
    // ========== 线程安全锁 ==========
    std::mutex prunepath_mutex;
    std::mutex optpath_mutex;
};

// ========== 工具函数（命名空间级别） ==========
namespace NavigationUtils
{
    /**
     * @brief 归一化角度到[-π, π]范围
     */
    double normalizeRadian(double angle);
    
    /**
     * @brief 绝对值限制函数
     * @param value 待限制的值
     * @param limit 限制阈值
     * @return 如果value在[-limit, limit]内返回0，否则返回原值
     */
    double ABS_limit(double value, double limit);
    
    /**
     * @brief 世界坐标转栅格坐标
     */
    bool world2Grid(double wx, double wy,
                    const nav_msgs::OccupancyGrid& costmap,
                    int& gx, int& gy);
    
    /**
     * @brief 栅格坐标转世界坐标
     */
    bool Grid2world(int gx, int gy,
                    const nav_msgs::OccupancyGrid& costmap,
                    double& wx, double& wy);
}

#endif  // CONTROLLER_H