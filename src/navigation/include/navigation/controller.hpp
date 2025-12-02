#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2/utils.h>
#include <iostream>

#include "utility.hpp"
#include "cubic_spline/cubic_spline_ros.h"

/**
 * @brief 导航控制器类
 * 
 * 负责路径跟踪和速度控制
 */
class Controller
{
public:
    Controller();
    ~Controller();

    // ========== ROS回调函数 ==========
    void GlobalPathCallback(const nav_msgs::PathConstPtr& msg);
    void LocalizationStatusCallback(const std_msgs::BoolConstPtr& msg);

    // ========== 主要控制循环 ==========
    void Plan(const ros::TimerEvent& event);

    // ========== 路径处理函数 ==========
    void FindNearstPose(geometry_msgs::PoseStamped& robot_pose,
                       nav_msgs::Path& path,
                       int& prune_index,
                       double prune_ahead_dist);

    // ========== 速度控制函数 ==========
    void FollowTraj(const geometry_msgs::PoseStamped& robot_pose,
                    const nav_msgs::Path& traj,
                    geometry_msgs::Twist& cmd_vel);

private:
    // ========== ROS接口成员 ==========
    ros::Publisher cmd_vel_pub;
    ros::Publisher local_path_pub;
    ros::Subscriber act_command_sub;
    ros::Subscriber global_path_sub;
    ros::Timer plan_timer;

    // ========== TF和路径数据 ==========
    std::shared_ptr<tf::TransformListener> tf_listener;
    nav_msgs::Path global_path;
    
    // ========== 系统状态标志 ==========
    bool diverge = false;  // 是否定位发散
    bool plan = false;     // 是否有有效规划
    int prune_index = 0;   // 全局路径剪枝索引

    // ========== 控制参数 ==========
    double max_speed;
    double set_yaw_speed = 0;
    double p_value;
    int plan_freq;
    double goal_dist_tolerance;
    double prune_ahead_dist;
    std::string global_frame;
    
    // ========== 计算中间变量 ==========
    double yaw;
};

    /**
     * @brief 归一化角度到[-π, π]范围
     */
inline double normalizeRadian(double angle)
{
    double n_angle = std::fmod(angle, 2 * M_PI);
    n_angle = n_angle > M_PI ? n_angle - 2 * M_PI : 
              n_angle < -M_PI ? 2 * M_PI + n_angle : n_angle;
    return n_angle;
}
    
    /**
     * @brief 绝对值限制函数
     * @param value 待限制的值
     * @param limit 限制阈值
     * @return 如果value在[-limit, limit]内返回0，否则返回原值
     */
inline double ABS_limit(double value, double limit)
{
    if (value < limit && value > -limit)
    {
        return 0;
    }
    else
    {
        return value;
    }
}

#endif  // CONTROLLER_H
