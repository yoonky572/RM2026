/**
 * @file segmentation.cpp
 * @brief 障碍物检测器实现文件
 * 
 * 功能说明：
 * 1. 加载先验地图点云
 * 2. 订阅激光雷达扫描数据（支持 Livox 和标准点云格式）
 * 3. 将点云转换到地图坐标系
 * 4. 使用 KD 树查找最近邻，检测动态障碍物（距离地图较远的点）
 * 5. 发布障碍物点云和发散标志
 */

#include "segmentation/segmentation.hpp"
#include "segmentation/utility.hpp"
#include <pcl/common/transforms.h>

// ============================================================================
// 构造函数：初始化障碍物检测器
// ============================================================================
Obstacle_detector::Obstacle_detector()
    : prior_map(new pcl::PointCloud<pcl::PointXYZ>)
    , filtered_prior_map(new pcl::PointCloud<pcl::PointXYZ>)
    , obstacle(new pcl::PointCloud<pcl::PointXYZ>)
    , scan_sensor(new pcl::PointCloud<pcl::PointXYZ>)
    , scan_map(new pcl::PointCloud<pcl::PointXYZ>)
    , last_scan_sensor(new pcl::PointCloud<pcl::PointXYZ>)
{
    ros::NodeHandle nh("~");

    // ========== 1. 加载 ROS 参数 ==========
    // 障碍物检测参数
    nh.param<double>("distance_threshold", distance_threshold, 0.2);
    nh.param<double>("diverge_threshold", diverge_threshold, 0.5);
    
    // 地图和滤波参数
    nh.param<std::string>("map_path", map_path, "");
    nh.param<double>("leaf_size", leaf_size, 0.05);
    
    // 运行参数
    nh.param<int>("freq", freq, 10);
    nh.param<bool>("prior_map_pub_en", prior_map_pub_en, false);
    nh.param<bool>("use_livox_cloud", use_livox_cloud, false);
    
    // 外参参数（传感器相对于基座的位姿）
    nh.param<std::vector<double>>("extrinsic_T", extrinT, std::vector<double>());
    nh.param<std::vector<double>>("extrinsic_R", extrinR, std::vector<double>());
    
    // IMU 外参参数（IMU 相对于基座的位姿）
    nh.param<std::vector<double>>("IMU_extrinsic_T", IMU_extrinT, std::vector<double>());
    nh.param<std::vector<double>>("IMU_extrinsic_R", IMU_extrinR, std::vector<double>());

    // ========== 2. 构建传感器到基座坐标系的变换矩阵 ==========
    // 解析外参：平移向量和旋转矩阵（3x3 矩阵按行展开为 9 个元素）
    Eigen::Vector3d translation(extrinT.data());
    Eigen::Matrix3d rotation;
    rotation << extrinR[0], extrinR[1], extrinR[2],
                extrinR[3], extrinR[4], extrinR[5],
                extrinR[6], extrinR[7], extrinR[8];
    
    // 解析 IMU 外参：平移向量和旋转矩阵
    Eigen::Vector3d imu_translation(IMU_extrinT.data());
    Eigen::Matrix3d imu_rotation;
    imu_rotation << IMU_extrinR[0], IMU_extrinR[1], IMU_extrinR[2],
                    IMU_extrinR[3], IMU_extrinR[4], IMU_extrinR[5],
                    IMU_extrinR[6], IMU_extrinR[7], IMU_extrinR[8];
    
    // 根据使用的点云类型选择不同的外参
    T_baselink_sensor = Eigen::Isometry3d::Identity();
    
    if (use_livox_cloud == 1) {
        // 使用 Livox 雷达时，使用直接的外参
        T_baselink_sensor.linear() = rotation;
        T_baselink_sensor.translation() = translation;
    } else {
        // 使用标准点云时，使用 IMU 外参
        // T_baselink_sensor 为 IMU 相对于 base_link 的坐标变换矩阵
        // Pb = T_baselink_sensor * Pa（将传感器坐标系中的点 Pa 转换到 base_link 坐标系）
        T_baselink_sensor.linear() = imu_rotation;
        T_baselink_sensor.translation() = imu_translation;
    }

    // ========== 3. 初始化 ROS 订阅器和发布器 ==========
    // 根据点云类型选择不同的订阅话题和回调函数
    if (use_livox_cloud == 1) {
        // Livox 雷达点云订阅
        // 注意：Livox_Scan_Callback 的频率取决于 /livox/lidar 话题的发布频率，由雷达硬件决定
        scan_sub = nh.subscribe("/livox/lidar", 5, 
                                &Obstacle_detector::Livox_Scan_Callback, this);
    } else {
        // 标准点云订阅
        scan_sub = nh.subscribe("/cloud_registered", 5, 
                                &Obstacle_detector::Standard_Scan_Callback, this);
    }
    
    // 发布器初始化
    diverge_pub = nh.advertise<std_msgs::Bool>("/diverge", 10);
    obstacle_pub = nh.advertise<sensor_msgs::PointCloud2>("obstacle", 10);
    prior_map_pub = nh.advertise<sensor_msgs::PointCloud2>("prior_map", 10);
    
    // TF 监听器初始化
    tf_listener = std::make_shared<tf::TransformListener>();
    
    // 定时器初始化：定期执行障碍物检测
    run_timer = nh.createTimer(ros::Duration(1.0 / freq), 
                               &Obstacle_detector::timer, this);

    // ========== 4. 加载并预处理先验地图 ==========
    // 加载 PCD 地图文件
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path, *prior_map) == -1) {
        ROS_ERROR("[Obstacle_detector] Failed to load PCD file: %s", map_path.c_str());
        ros::shutdown();
        return;
    }
    
    // 体素滤波：降低地图点云密度，提高 KD 树查询效率
    // 体素滤波可以保证点云有序存储，有利于 KD 树构建
    pcl::VoxelGrid<pcl::PointXYZ> downsample;
    downsample.setInputCloud(prior_map);
    downsample.setLeafSize(leaf_size, leaf_size, leaf_size);
    downsample.filter(*filtered_prior_map);
    
    // 构建 KD 树：用于快速最近邻搜索
    kdtree.setInputCloud(filtered_prior_map);
    
    ROS_INFO("[Obstacle_detector] Map loaded: %zu points (filtered from %zu points)", 
             filtered_prior_map->size(), prior_map->size());
}

// ============================================================================
// 障碍物检测函数
// ============================================================================
void Obstacle_detector::detect(const pcl::PointCloud<pcl::PointXYZ>::Ptr& scan_map)
{
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // scan_map 为当前帧点云（已经在 map 坐标系下）
    // 对每个点查找最近邻，如果距离超过阈值，认为是动态障碍物
    
    for (const auto& pt : scan_map->points) {
        std::vector<int> indices(1);           // 最近邻索引
        std::vector<float> sqr_distance(1);    // 最近邻距离的平方
        
        // 在 KD 树中查找最近邻点
        if (kdtree.nearestKSearch(pt, 1, indices, sqr_distance) > 0) {
            // 计算欧氏距离
            double distance = std::sqrt(sqr_distance[0]);
            
            // 如果距离超过阈值，认为是障碍物点
            if (distance > distance_threshold) {
                obstacle->push_back(pt);
            }
        }
    }
    
    // 计算障碍物点百分比
    double obstacle_percentage = 0.0;
    if (scan_map->size() > 0) {
        obstacle_percentage = static_cast<double>(obstacle->size()) / scan_map->size();
    }
    
    // 判断是否发散：障碍物点过多表示定位可能失败
    if (obstacle_percentage > diverge_threshold) {
        diverge.data = true;
    } else {
        diverge.data = false;
    }
    
    // 输出性能统计信息
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> duration = end_time - start_time;
    std::cout << "[Obstacle_detector] Detect function took " 
              << duration.count() << " milliseconds" << std::endl;
    std::cout << "[Obstacle_detector] Obstacle point percentage: " 
              << obstacle_percentage * 100.0 << "%" << std::endl;
}

// ============================================================================
// 定时器回调函数：定期执行障碍物检测流程
// ============================================================================
void Obstacle_detector::timer(const ros::TimerEvent& event)
{
    // 清空上一帧的障碍物点云
    obstacle->clear();
    
    // ========== 1. 获取机器人当前位姿 ==========
    geometry_msgs::PoseStamped robot_pose;
    
    // 根据使用的点云类型选择不同的坐标系查询
    if (use_livox_cloud == 1) {
        // 使用 Livox 时，查询 base_link 到 map 的变换
        GetTargetPose(tf_listener, "map", "base_link", robot_pose, last_scan_timestamp);
    } else {
        // 使用标准点云时，查询 odom 到 map 的变换
        // 查询从 odom 坐标系到 map 坐标系的坐标变换，并将结果保存在 robot_pose 中
        GetTargetPose(tf_listener, "map", "odom", robot_pose, last_scan_timestamp);
    }
    
    // ========== 2. 构建坐标变换矩阵 ==========
    // 提取位置和姿态
    Eigen::Vector3d position(
        robot_pose.pose.position.x,
        robot_pose.pose.position.y,
        robot_pose.pose.position.z);
    
    Eigen::Quaterniond orientation(
        robot_pose.pose.orientation.w,
        robot_pose.pose.orientation.x,
        robot_pose.pose.orientation.y,
        robot_pose.pose.orientation.z);
    orientation.normalize();
    
    // 构建从目标坐标系（odom 或 base_link）到 map 的变换矩阵
    Eigen::Isometry3d T_map_target;
    T_map_target.linear() = orientation.toRotationMatrix();
    T_map_target.translation() = position;
    
    // 构建从传感器坐标系到 map 坐标系的变换矩阵
    // T_map_sensor = T_map_target * T_baselink_sensor
    Eigen::Affine3d T_map_sensor = T_map_target * T_baselink_sensor;
    
    // ========== 3. 获取最新扫描点云 ==========
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_local(new pcl::PointCloud<pcl::PointXYZ>());
    
    {
        // 使用互斥锁保护共享数据
        // 因为 Livox_Scan_Callback 和 Standard_Scan_Callback 也使用 last_scan_sensor
        // 锁保证 last_scan_sensor 只在一个线程使用
        std::lock_guard<std::mutex> lock(scan_mutex);
        
        if (last_scan_sensor->size() != 0) {
            // 复制回调函数中获取的最新点云
            *scan_local = *last_scan_sensor;
        }
    }
    
    // ========== 4. 处理点云并检测障碍物 ==========
    if (scan_local->size() != 0) {
        // 将点云从传感器坐标系转换到 map 坐标系，便于与先验地图对比
        pcl::transformPointCloud(*scan_local, *scan_map, T_map_sensor);
        
        // 执行障碍物检测
        detect(scan_map);
        
        // ========== 5. 发布障碍物点云 ==========
        sensor_msgs::PointCloud2 obstacle_ros;
        pcl::toROSMsg(*obstacle, obstacle_ros);  // 从点云格式转化为 ROS 可识别格式
        obstacle_ros.header.frame_id = "map";
        obstacle_ros.header.stamp = ros::Time::now();
        obstacle_pub.publish(obstacle_ros);
        
        // ========== 6. 发布路径偏离标志 ==========
        diverge_pub.publish(diverge);
    }
    
    // ========== 7. 发布先验地图（如果启用，用于可视化） ==========
    if (prior_map_pub_en) {
        sensor_msgs::PointCloud2 prior_map_ros;
        pcl::toROSMsg(*filtered_prior_map, prior_map_ros);
        prior_map_ros.header.frame_id = "map";
        prior_map_ros.header.stamp = ros::Time::now();
        prior_map_pub.publish(prior_map_ros);
    }
}

// ============================================================================
// Livox 雷达点云回调函数
// ============================================================================
void Obstacle_detector::Livox_Scan_Callback(const livox_ros_driver2::CustomMsg::ConstPtr& msg)
{
    // 使用互斥锁保护共享数据
    std::lock_guard<std::mutex> lock(scan_mutex);
    
    // scan_sensor 用于不断接收最新点云
    // last_scan_sensor 用于储存这帧点云，供障碍物检测使用
    *last_scan_sensor = *scan_sensor;
    
    // 处理 Livox 消息格式，自动覆盖 scan_sensor
    livox_msg_handler(msg, scan_sensor);
    
    // 更新时间戳
    last_scan_timestamp = scan_timestamp;
    scan_timestamp = msg->header.stamp;
}

// ============================================================================
// 标准点云回调函数
// ============================================================================
void Obstacle_detector::Standard_Scan_Callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    // 使用互斥锁保护共享数据
    std::lock_guard<std::mutex> lock(scan_mutex);
    
    // 保存上一帧点云
    *last_scan_sensor = *scan_sensor;
    
    // 处理标准点云消息格式
    standard_msg_handler(msg, scan_sensor);
    
    // 更新时间戳
    last_scan_timestamp = scan_timestamp;
    scan_timestamp = msg->header.stamp;
}

// ============================================================================
// 主函数
// ============================================================================
int main(int argc, char** argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "obstacle_detector");
    
    // 创建障碍物检测器实例
    Obstacle_detector obstacle_detector;
    
    // 进入 ROS 事件循环
    ros::spin();
    
    return 0;
}
