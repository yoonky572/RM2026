#pragma once

#include <mutex>
#include <queue>
#include <memory>
#include <chrono>

// ROS includes
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>

// PCL includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/search/kdtree.h>

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
 * @brief Relocalization 类：基于点云配准的机器人重定位系统
 * 
 * 功能说明：
 * 1. 加载先验地图和障碍物点云
 * 2. 订阅激光雷达扫描和障碍物点云
 * 3. 使用 GICP 进行点云配准，实现机器人定位
 * 4. 当定位失败时，使用 FPFH 特征进行粗配准
 * 5. 发布定位结果和 TF 变换
 */
class Relocalization {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
    Relocalization();
    ~Relocalization() = default;

private:
    /**
     * @brief 定时器回调函数：执行点云配准和定位
     */
    void timer(const ros::TimerEvent& event);

    /**
     * @brief 执行点云配准
     * @param source 源点云（当前扫描）
     * @param obs 障碍物点云
     * @param leaf_size 体素滤波的叶子大小
     * @param threshold 配准误差阈值
     */
    void registration(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source, 
                     const pcl::PointCloud<pcl::PointXYZ>::Ptr& obs, 
                     double leaf_size, 
                     double threshold);

    /**
     * @brief 计算 FPFH 特征
     * @param input_cloud 输入点云
     * @param tree KD树用于最近邻搜索
     * @param fpfh 输出的 FPFH 特征
     */
    void computeFpfhFeature(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                            pcl::search::KdTree<pcl::PointXYZ>::Ptr& tree,
                            pcl::PointCloud<pcl::FPFHSignature33>::Ptr& fpfh);

    /**
     * @brief 使用 SAC-IA 进行粗配准
     * @param source_cloud 源点云
     * @param target_cloud 目标点云
     * @return 配准后的点云（未使用，保留接口兼容性）
     */
    pcl::PointCloud<pcl::PointXYZ> sacIaCompute(
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud);

    /**
     * @brief 发布调试点云（如果启用调试模式）
     */
    void publishDebugClouds();

    /**
     * @brief 发布 TF 变换
     * @param transform 变换矩阵
     */
    void publishTransform(const Eigen::Isometry3d& transform);

    // ========== 回调函数 ==========
    void obsCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void scanCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void divergeCallback(const std_msgs::Bool::ConstPtr& msg);
    void bodyPoseCallback(const geometry_msgs::PoseStamped& msg);

    // ========== ROS 相关 ==========
    ros::NodeHandle nh_;                          // 节点句柄
    ros::Subscriber scan_sub_;                    // 扫描点云订阅器
    ros::Subscriber obs_sub_;                     // 障碍物点云订阅器
    ros::Subscriber initial_pose_sub_;            // 初始位姿订阅器
    ros::Subscriber diverge_sub_;                 // 发散标志订阅器
    ros::Subscriber odom_sub_;                    // 里程计订阅器
    ros::Publisher target_pub_;                   // 目标点云发布器（调试用）
    ros::Publisher source_pub_;                   // 源点云发布器（调试用）
    ros::Publisher align_pub_;                    // 配准后点云发布器（调试用）
    ros::Publisher localize_success_pub_;         // 定位成功标志发布器
    ros::Timer run_timer_;                        // 定时器

    // ========== TF 相关 ==========
    tf2_ros::Buffer tf2_buffer_;                  // TF 缓冲区
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;  // TF 监听器
    tf2_ros::TransformBroadcaster tf_broadcaster_;             // TF 广播器
    geometry_msgs::TransformStamped T_vice_map_odom_;  // vice_map 到 odom 的变换

    // ========== 点云数据 ==========
    pcl::PointCloud<pcl::PointXYZ>::Ptr prior_map_;           // 先验地图
    pcl::PointCloud<pcl::PointXYZ>::Ptr prior_obs_;           // 先验障碍物点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_prior_map_;  // 滤波后的先验地图
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_prior_obs_;  // 滤波后的先验障碍物点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_;                // 当前扫描点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr obs_;                  // 当前障碍物点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_odom_;            // 转换到 odom 坐标系的扫描点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_;              // 配准后的点云（调试用）

    // ========== 点云协方差和特征 ==========
    pcl::PointCloud<pcl::PointCovariance>::Ptr source_cov_;   // 源点云协方差
    pcl::PointCloud<pcl::PointCovariance>::Ptr target_cov_;    // 目标点云协方差
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_fpfh_; // 源点云 FPFH 特征
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_fpfh_;   // 目标点云 FPFH 特征
    pcl::PointCloud<pcl::Normal>::Ptr point_normal_;           // 点云法向量

    // ========== KD 树 ==========
    std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> target_tree_;  // 目标点云 KD 树
    std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> source_tree_; // 源点云 KD 树

    // ========== 配准器 ==========
    std::shared_ptr<small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>> register_;

    // ========== 特征估计器 ==========
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> est_normal_;  // 法向量估计器
    pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> est_fpfh_;  // FPFH 特征估计器

    // ========== 变换矩阵 ==========
    Eigen::Isometry3d T_odom_lidar_;          // odom 到 lidar 的变换
    Eigen::Isometry3d T_map_scan_;             // map 到 scan 的变换（配准结果）
    Eigen::Isometry3d previous_icp_result_;    // 上一次 ICP 配准结果
    Eigen::Isometry3d fpfh_result_;            // FPFH 粗配准结果
    Eigen::Isometry3d T_relocalize_;           // 重定位变换

    // ========== 线程安全 ==========
    std::mutex scan_mutex_;                    // 扫描点云互斥锁
    std::mutex obs_mutex_;                     // 障碍物点云互斥锁
    std::queue<pcl::PointCloud<pcl::PointXYZ>> scan_queue_;  // 扫描点云队列
    std::queue<pcl::PointCloud<pcl::PointXYZ>> obs_queue_;    // 障碍物点云队列

    // ========== 状态标志 ==========
    bool debug_en_;                            // 是否启用调试模式
    bool lost_;                                // 是否丢失定位
    bool localize_success_;                    // 定位是否成功
    std_msgs::Bool diverge_;                   // 发散标志

    // ========== 参数 ==========
    std::string pcd_path_;                     // 地图 PCD 文件路径
    std::string obs_path_;                     // 障碍物 PCD 文件路径
    int freq_;                                 // 运行频率
    int max_frame_;                            // 最大缓存帧数
    int num_threads_;                          // 线程数
    int num_neighbors_;                        // 最近邻数量
    int yaw_bias_cnt_;                         // 偏航角偏差计数器
    double leaf_size_;                         // 体素滤波叶子大小
    double map_leaf_size_;                     // 地图体素滤波叶子大小
    double max_dist_sq_;                       // 最大距离平方
    double max_iterations_;                    // 最大迭代次数
    double diverge_threshold_;                 // 发散阈值
    double previous_error_;                    // 上一次配准误差
};
