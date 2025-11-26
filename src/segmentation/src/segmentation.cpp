#include "segmentation/segmentation.hpp"
#include "segmentation/utility.hpp"
#include <pcl/filters/conditional_removal.h>

Obstacle_detector::Obstacle_detector()
    : prior_map_(new pcl::PointCloud<pcl::PointXYZ>())
    , filtered_prior_map_(new pcl::PointCloud<pcl::PointXYZ>())
    , obstacle_(new pcl::PointCloud<pcl::PointXYZ>())
    , obs_(new pcl::PointCloud<pcl::PointXYZ>())
    , scan_sensor_(new pcl::PointCloud<pcl::PointXYZ>())
    , last_scan_sensor_(new pcl::PointCloud<pcl::PointXYZ>())
    , scan_map_(new pcl::PointCloud<pcl::PointXYZ>())
    , cropped_scan_(new pcl::PointCloud<pcl::PointXYZ>())
    , cropped_map_(new pcl::PointCloud<pcl::PointXYZ>())
    , cropped_obs_(new pcl::PointCloud<pcl::PointXYZ>())
    , register_(std::make_shared<small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>())
    , previous_icp_result_(Eigen::Isometry3d::Identity())
    , high_match_(false)
    , diverge_()
    , match_()
    , nh_("~")
    , box_center_(0.0, 0.0, 0.0)
{
    // ========== 加载参数 ==========
    nh_.param<double>("distance_threshold", distance_threshold_, 0.2);
    nh_.param<std::string>("map_path", map_path_, "");
    nh_.param<int>("freq", freq_, 10);
    nh_.param<double>("leaf_size", leaf_size_, 0.05);
    nh_.param<float>("box_size", box_size_, 1.5);
    nh_.param<bool>("prior_map_pub_en", prior_map_pub_en_, false);
    nh_.param<bool>("use_livox_cloud", use_livox_cloud_, false);
    nh_.param<double>("diverge_threshold", diverge_threshold_, 0.5);
    nh_.param<double>("high_match_threshold", high_match_threshold_, 0.2);
    nh_.param<double>("lidar_height", lidar_height_, 0.145);
    nh_.param<double>("lidar_roll", lidar_roll_, -0.349);
    nh_.param<double>("lidar_y", lidar_y_, 0.11);
    nh_.param<bool>("debug_en", debug_en_, false);
    nh_.param<bool>("align_en", align_en_, false);
    nh_.param<double>("max_dist_sq", max_dist_sq_, 1.0);
    nh_.param<double>("max_iterations", max_iterations_, 100.0);

    // ========== 配置配准器 ==========
    register_->reduction.num_threads = 4;
    register_->rejector.max_dist_sq = max_dist_sq_;
    register_->criteria.rotation_eps = 1e-7;
    register_->optimizer.max_iterations = static_cast<int>(max_iterations_);

    // ========== 初始化订阅器和发布器 ==========
    scan_sub_ = nh_.subscribe("/cloud_registered_body", 5, 
                               &Obstacle_detector::scanCallback, this);
    obs_sub_ = nh_.subscribe("/ground_segmentation/obstacle_cloud", 1, 
                             &Obstacle_detector::obsCallback, this);

    diverge_pub_ = nh_.advertise<std_msgs::Bool>("/diverge", 10);
    match_pub_ = nh_.advertise<std_msgs::Bool>("/match", 10);
    obstacle_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("obstacle", 10);
    prior_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("prior_map", 10);
    aligned_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("aligned", 10);
    body_frame_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/body_pose", 10);

    // ========== 创建定时器和 TF 监听器 ==========
    tf_listener_ = std::make_shared<tf::TransformListener>();
    run_timer_ = nh_.createTimer(ros::Duration(1.0 / freq_), 
                                  &Obstacle_detector::timer, this);

    // ========== 加载地图文件 ==========
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path_, *prior_map_) == -1) {
        ROS_ERROR("[Obstacle_detector] Failed to load PCD file: %s", map_path_.c_str());
        ros::shutdown();
        return;
    }
    ROS_INFO("[Obstacle_detector] Successfully loaded PCD file: %s (%zu points)", 
             map_path_.c_str(), prior_map_->size());

    // ========== 变换地图到正确坐标系 ==========
    // 如果用扫描得到的点云当作目标，要先把点云转换到地图坐标系，而不是建图时 pointlio 的起点坐标系
    Eigen::Affine3d T_map_vice_map = Eigen::Affine3d::Identity();
    Eigen::AngleAxis roll(lidar_roll_, Eigen::Vector3d::UnitY());
    T_map_vice_map.linear() = roll.toRotationMatrix();
    T_map_vice_map.translation().z() = lidar_height_;
    T_map_vice_map.translation().x() = lidar_y_;
    pcl::transformPointCloud(*prior_map_, *prior_map_, T_map_vice_map);

    // ========== 体素滤波地图 ==========
    pcl::VoxelGrid<pcl::PointXYZ> downsample;
    downsample.setInputCloud(prior_map_);
    downsample.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    downsample.filter(*filtered_prior_map_);

    // ========== 构建 KD 树 ==========
    kdtree_.setInputCloud(filtered_prior_map_);

    // ========== 初始化裁剪区域 ==========
    box_center_.x() = 0.0;
    box_center_.y() = 0.0;
    box_center_.z() = 0.0;
    cloudCrop(filtered_prior_map_, box_center_, box_size_, cropped_map_);

    ROS_INFO("[Obstacle_detector] Initialization completed");
}

void Obstacle_detector::timer(const ros::TimerEvent& event)
{
    auto start_time = std::chrono::high_resolution_clock::now();
    obstacle_->clear();

    // ========== 获取机器人位姿 ==========
    geometry_msgs::PoseStamped init_pose;
    geometry_msgs::PoseStamped robot_pose;
    geometry_msgs::PoseStamped body_pose;

    // 根据是否使用 Livox 点云选择不同的坐标系
    if (use_livox_cloud_) {
        GetTargetPose(tf_listener_, "map", "base_link", init_pose, last_scan_timestamp_);
    } else {
        GetTargetPose(tf_listener_, "map", "body", init_pose, last_scan_timestamp_);
    }
    GetTargetPose(tf_listener_, "map", "base_link", robot_pose, last_scan_timestamp_);
    GetTargetPose(tf_listener_, "vice_map", "body", body_pose, last_scan_timestamp_);

    // ========== 构建变换矩阵 ==========
    Eigen::Vector3d robot_position(
        robot_pose.pose.position.x,
        robot_pose.pose.position.y,
        robot_pose.pose.position.z);

    Eigen::Vector3d position(
        init_pose.pose.position.x,
        init_pose.pose.position.y,
        init_pose.pose.position.z);

    Eigen::Quaterniond orientation(
        init_pose.pose.orientation.w,
        init_pose.pose.orientation.x,
        init_pose.pose.orientation.y,
        init_pose.pose.orientation.z);
    orientation.normalize();

    // target 是 baselink（用原始点云）或 odom（用 pointlio 点云）
    Eigen::Isometry3d T_map_target;
    T_map_target.linear() = orientation.toRotationMatrix();
    T_map_target.translation() = position;

    Eigen::Affine3d T_map_sensor = T_map_target;

    // ========== 获取点云数据 ==========
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_local(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr obs_map(new pcl::PointCloud<pcl::PointXYZ>());

    {
        std::lock_guard<std::mutex> lock(scan_mutex_);
        if (last_scan_sensor_->size() != 0) {
            *scan_local = *last_scan_sensor_;
        }
    }

    {
        std::lock_guard<std::mutex> lock(obs_mutex_);
        pcl::transformPointCloud(*obs_, *obs_map, T_map_sensor);
    }

    // ========== 处理扫描点云 ==========
    if (scan_local->size() != 0) {
        // 转换到地图坐标系
        pcl::transformPointCloud(*scan_local, *scan_map_, T_map_sensor);

        // ========== 更新裁剪区域 ==========
        // 如果机器人移动超过阈值，更新裁剪区域中心
        if ((std::abs(box_center_.x() - robot_position.x()) > box_size_ - 1.0) ||
            (std::abs(box_center_.y() - robot_position.y()) > box_size_ - 1.0)) {
            box_center_ = robot_position;
            cloudCrop(filtered_prior_map_, box_center_, box_size_, cropped_map_);
        }

        // ========== 体素滤波和裁剪 ==========
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_scan, filtered_obs;
        filtered_scan = small_gicp::voxelgrid_sampling_omp(*scan_map_, leaf_size_);
        filtered_obs = small_gicp::voxelgrid_sampling_omp(*obs_map, leaf_size_);
        cloudCrop(filtered_scan, box_center_, box_size_, cropped_scan_);
        cloudCrop(filtered_obs, box_center_, box_size_, cropped_obs_);

        // ========== 执行配准（如果启用） ==========
        if (cropped_scan_->size() != 0) {
            if (align_en_) {
                // 生成协方差点云
                target_cov_ = small_gicp::voxelgrid_sampling_omp<
                    pcl::PointCloud<pcl::PointXYZ>,
                    pcl::PointCloud<pcl::PointCovariance>>(*cropped_map_, leaf_size_, 4);
                small_gicp::estimate_covariances_omp(*target_cov_, 2, 4);
                target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
                    target_cov_, small_gicp::KdTreeBuilderOMP(4));

                source_cov_ = small_gicp::voxelgrid_sampling_omp<
                    pcl::PointCloud<pcl::PointXYZ>,
                    pcl::PointCloud<pcl::PointCovariance>>(*cropped_scan_, leaf_size_, 4);
                small_gicp::estimate_covariances_omp(*source_cov_, 2, 4);
                source_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
                    source_cov_, small_gicp::KdTreeBuilderOMP(4));

                // 执行配准
                small_gicp::RegistrationResult result;
                result = register_->align(*target_cov_, *source_cov_, *target_tree_, previous_icp_result_);
                
                // 应用配准结果
                Eigen::Affine3d T(result.T_target_source);
                pcl::transformPointCloud(*cropped_obs_, *cropped_obs_, T);
                pcl::transformPointCloud(*cropped_scan_, *cropped_scan_, T);
            }

            auto mid_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> duration_mid = mid_time - start_time;
            ROS_DEBUG("[Obstacle_detector] Transform & downsample & registration took %.2f ms", 
                      duration_mid.count());

            // ========== 检测障碍物 ==========
            detect(cropped_scan_);

            // ========== 发布配准后的点云（如果启用配准和调试） ==========
            if (align_en_) {
                sensor_msgs::PointCloud2 aligned_ros;
                pcl::toROSMsg(*cropped_scan_, aligned_ros);
                aligned_ros.header.frame_id = "map";
                aligned_ros.header.stamp = ros::Time::now();
                aligned_pub_.publish(aligned_ros);
            }

            // ========== 发布障碍物点云 ==========
            // 转换回 body 坐标系
            pcl::transformPointCloud(*obstacle_, *obstacle_, T_map_sensor.inverse());
            sensor_msgs::PointCloud2 obstacle_ros;
            pcl::toROSMsg(*obstacle_, obstacle_ros);
            obstacle_ros.header.frame_id = "body";
            obstacle_ros.header.stamp = ros::Time::now();
            obstacle_pub_.publish(obstacle_ros);

            // ========== 发布匹配和发散标志 ==========
            diverge_pub_.publish(diverge_);
            match_pub_.publish(match_);

            // ========== 发布机器人位姿（如果匹配度高） ==========
            // 发布未漂移的里程计信息供重定位使用
            if (high_match_) {
                body_frame_pub_.publish(body_pose);
            }
        }
    }

    // ========== 发布先验地图（如果启用） ==========
    if (prior_map_pub_en_) {
        sensor_msgs::PointCloud2 prior_map_ros;
        pcl::toROSMsg(*cropped_map_, prior_map_ros);
        prior_map_ros.header.frame_id = "map";
        prior_map_ros.header.stamp = ros::Time::now();
        prior_map_pub_.publish(prior_map_ros);
    }
}

void Obstacle_detector::detect(const pcl::PointCloud<pcl::PointXYZ>::Ptr& scan_map)
{
    auto start_time = std::chrono::high_resolution_clock::now();

    ROS_DEBUG("[Obstacle_detector] Detect scan size: %zu", scan_map->size());

    // ========== 检测动态障碍物 ==========
    // 对每个点查找最近邻，如果距离超过阈值，认为是障碍物
    for (const auto& pt : scan_map->points) {
        std::vector<int> indices(1);
        std::vector<float> sqr_distances(1);

        // 查找最近邻
        if (kdtree_.nearestKSearch(pt, 1, indices, sqr_distances) > 0) {
            // 如果距离超过阈值，认为是动态障碍物
            if (std::sqrt(sqr_distances[0]) > distance_threshold_) {
                obstacle_->push_back(pt);
            }
        }
    }

    // ========== 计算障碍物点百分比 ==========
    double obstacle_percentage = 0.0;
    if (scan_map->size() > 0) {
        obstacle_percentage = static_cast<double>(obstacle_->size()) / scan_map->size();
    }

    // ========== 判断匹配状态 ==========
    if (obstacle_percentage > diverge_threshold_) {
        // 障碍物点过多，认为匹配失败
        match_.data = false;
        high_match_ = false;
        
        // 如果匹配失败持续超过 3 秒，认为发散
        std::chrono::duration<double, std::milli> diverge_duration = 
            std::chrono::high_resolution_clock::now() - match_time_;
        if (diverge_duration > std::chrono::milliseconds(3000)) {
            diverge_.data = true;
        }
    } else {
        // 匹配成功
        if (obstacle_percentage <= high_match_threshold_) {
            high_match_ = true;
        } else {
            high_match_ = false;
        }
        match_.data = true;
        match_time_ = std::chrono::high_resolution_clock::now();
        diverge_.data = false;
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> duration = end_time - start_time;
    
    if (debug_en_) {
        ROS_INFO("[Obstacle_detector] Detect function took %.2f ms", duration.count());
        ROS_INFO("[Obstacle_detector] Obstacle point percentage: %.2f%%", 
                 obstacle_percentage * 100.0);
    }
}

void Obstacle_detector::cloudCrop(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                   const Eigen::Vector3d& pos,
                                   float box_size,
                                   pcl::PointCloud<pcl::PointXYZ>::Ptr& result)
{
    // ========== 设置裁剪边界 ==========
    float x_min = pos.x() - box_size;
    float x_max = pos.x() + box_size;
    float y_min = pos.y() - box_size;
    float y_max = pos.y() + box_size;
    float z_min = pos.z() - box_size;
    float z_max = pos.z() + 1.5;  // Z 方向上限固定为 1.5 米

    // ========== 使用条件滤波器裁剪点云 ==========
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr cond(new pcl::ConditionAnd<pcl::PointXYZ>);
    cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT, x_min)));
    cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, x_max)));
    cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, y_min)));
    cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, y_max)));
    cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, z_min)));
    cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, z_max)));

    pcl::ConditionalRemoval<pcl::PointXYZ> cond_filter;
    cond_filter.setInputCloud(cloud);
    cond_filter.setCondition(cond);
    cond_filter.filter(*result);

    ROS_DEBUG("[Obstacle_detector] Cropped cloud size: %zu (from %zu)", 
              result->size(), cloud->size());
}

void Obstacle_detector::scanCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(scan_mutex_);
    *last_scan_sensor_ = *scan_sensor_;
    standard_msg_handler(msg, scan_sensor_);
    last_scan_timestamp_ = scan_timestamp_;
    scan_timestamp_ = msg->header.stamp;
}

void Obstacle_detector::obsCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ> temp_cloud;
    std::lock_guard<std::mutex> lock(obs_mutex_);
    obs_->clear();
    int size = msg->height * msg->width;
    obs_->resize(size);
    pcl::fromROSMsg(*msg, temp_cloud);
    *obs_ = temp_cloud;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_detector");
    Obstacle_detector obstacle_detector;
    ros::spin();
    return 0;
}
