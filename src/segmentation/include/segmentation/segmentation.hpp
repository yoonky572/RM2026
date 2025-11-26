#pragma once

#include <mutex>
#include <memory>
#include <chrono>
#include <string>

// ROS includes
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>

// PCL includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/common/transforms.h>

// Eigen includes
#include <Eigen/Geometry>
#include <Eigen/Dense>

// small_gicp includes
#include "small_gicp/ann/kdtree_omp.hpp"
#include "small_gicp/factors/gicp_factor.hpp"
#include "small_gicp/pcl/pcl_point.hpp"
#include "small_gicp/util/downsampling_omp.hpp"
#include "small_gicp/registration/reduction_omp.hpp"
#include "small_gicp/registration/registration.hpp"
#include "small_gicp/pcl/pcl_registration.hpp"

/**
 * @brief Obstacle_detector 类：基于点云配准的动态障碍物检测器
 * 
 * 功能说明：
 * 1. 加载先验地图点云
 * 2. 订阅激光雷达扫描和障碍物点云
 * 3. 将点云转换到地图坐标系
 * 4. 使用 KD 树查找最近邻，检测动态障碍物（距离地图较远的点）
 * 5. 可选的点云配准功能，提高检测精度
 * 6. 发布障碍物点云、匹配状态等信息
 */
class Obstacle_detector {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Obstacle_detector();
    ~Obstacle_detector() = default;

private:
    /**
     * @brief 定时器回调函数：执行障碍物检测
     */
    void timer(const ros::TimerEvent& event);

    /**
     * @brief 检测动态障碍物
     * @param scan_map 地图坐标系下的扫描点云
     */
    void detect(const pcl::PointCloud<pcl::PointXYZ>::Ptr& scan_map);

    /**
     * @brief 裁剪点云到指定区域
     * @param cloud 输入点云
     * @param pos 区域中心位置
     * @param box_size 区域大小（米）
     * @param result 输出的裁剪后点云
     */
    void cloudCrop(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                   const Eigen::Vector3d& pos,
                   float box_size,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr& result);

    // ========== 回调函数 ==========
    void scanCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void obsCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

    // ========== ROS 相关 ==========
    ros::NodeHandle nh_;                          // 节点句柄
    ros::Subscriber scan_sub_;                     // 扫描点云订阅器
    ros::Subscriber obs_sub_;                      // 障碍物点云订阅器
    ros::Publisher obstacle_pub_;                  // 障碍物点云发布器
    ros::Publisher prior_map_pub_;                 // 先验地图发布器
    ros::Publisher aligned_pub_;                   // 配准后点云发布器（调试用）
    ros::Publisher diverge_pub_;                   // 发散标志发布器
    ros::Publisher match_pub_;                     // 匹配标志发布器
    ros::Publisher body_frame_pub_;                // 机器人位姿发布器
    ros::Timer run_timer_;                        // 定时器

    // ========== TF 相关 ==========
    std::shared_ptr<tf::TransformListener> tf_listener_;  // TF 监听器

    // ========== 点云数据 ==========
    pcl::PointCloud<pcl::PointXYZ>::Ptr prior_map_;           // 先验地图
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_prior_map_;   // 滤波后的先验地图
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_;             // 检测到的障碍物点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr obs_;                  // 当前障碍物点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_sensor_;         // 传感器坐标系下的扫描点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr last_scan_sensor_;      // 上一次的扫描点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_map_;             // 地图坐标系下的扫描点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_scan_;          // 裁剪后的扫描点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_map_;          // 裁剪后的地图点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_obs_;          // 裁剪后的障碍物点云

    // ========== 点云协方差和 KD 树 ==========
    pcl::PointCloud<pcl::PointCovariance>::Ptr source_cov_;    // 源点云协方差
    pcl::PointCloud<pcl::PointCovariance>::Ptr target_cov_;    // 目标点云协方差
    std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> target_tree_;  // 目标点云 KD 树
    std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> source_tree_; // 源点云 KD 树
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;                   // 先验地图 KD 树

    // ========== 配准器 ==========
    std::shared_ptr<small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>> register_;
    Eigen::Isometry3d previous_icp_result_;                  // 上一次 ICP 配准结果

    // ========== 状态标志 ==========
    bool high_match_;                            // 高匹配度标志
    std_msgs::Bool diverge_;                     // 发散标志
    std_msgs::Bool match_;                       // 匹配标志
    std::chrono::time_point<std::chrono::high_resolution_clock> match_time_;  // 匹配时间

    // ========== 参数 ==========
    std::string map_path_;                       // 地图文件路径
    double distance_threshold_;                  // 障碍物检测距离阈值（米）
    double leaf_size_;                           // 体素滤波叶子大小（米）
    double diverge_threshold_;                   // 发散阈值（障碍物点百分比）
    double high_match_threshold_;                // 高匹配阈值（障碍物点百分比）
    double lidar_height_;                        // 激光雷达高度（米）
    double lidar_roll_;                          // 激光雷达俯仰角（弧度）
    double lidar_y_;                             // 激光雷达 Y 偏移（米）
    double max_dist_sq_;                         // 配准最大距离平方
    double max_iterations_;                      // 配准最大迭代次数
    float box_size_;                             // 裁剪区域大小（米）
    int freq_;                                   // 运行频率（Hz）
    bool prior_map_pub_en_;                      // 是否发布先验地图
    bool use_livox_cloud_;                       // 是否使用 Livox 点云（未使用）
    bool debug_en_;                              // 是否启用调试模式
    bool align_en_;                              // 是否启用配准

    // ========== 状态变量 ==========
    Eigen::Vector3d box_center_;                 // 裁剪区域中心
    ros::Time scan_timestamp_;                   // 扫描时间戳
    ros::Time last_scan_timestamp_;              // 上一次扫描时间戳

    // ========== 线程安全 ==========
    std::mutex scan_mutex_;                      // 扫描点云互斥锁
    std::mutex obs_mutex_;                       // 障碍物点云互斥锁
};

/**
 * @brief 将栅格坐标转换为世界坐标
 * @param gx 栅格坐标 x
 * @param gy 栅格坐标 y
 * @param costmap 代价地图
 * @param wx 输出的世界坐标 x
 * @param wy 输出的世界坐标 y
 * @return 转换是否成功
 */
inline bool Grid2world(int gx, int gy, const nav_msgs::OccupancyGrid& costmap,
                       double& wx, double& wy)
{
    if (costmap.info.width == 0 || costmap.info.height == 0) {
        return false;
    }

    wx = (gx + 0.5) * costmap.info.resolution + costmap.info.origin.position.x;
    wy = (gy + 0.5) * costmap.info.resolution + costmap.info.origin.position.y;

    return true;
}
