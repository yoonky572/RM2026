#include "relocalization/relocalization.hpp"
#include <small_gicp/util/downsampling_omp.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/ia_ransac.h>
#include <sstream>

Relocalization::Relocalization()
    : prior_map_(new pcl::PointCloud<pcl::PointXYZ>())
    , prior_obs_(new pcl::PointCloud<pcl::PointXYZ>())
    , filtered_prior_map_(new pcl::PointCloud<pcl::PointXYZ>())
    , filtered_prior_obs_(new pcl::PointCloud<pcl::PointXYZ>())
    , scan_(new pcl::PointCloud<pcl::PointXYZ>())
    , obs_(new pcl::PointCloud<pcl::PointXYZ>())
    , scan_odom_(new pcl::PointCloud<pcl::PointXYZ>())
    , aligned_(new pcl::PointCloud<pcl::PointXYZ>())
    , source_fpfh_(new pcl::PointCloud<pcl::FPFHSignature33>())
    , target_fpfh_(new pcl::PointCloud<pcl::FPFHSignature33>())
    , point_normal_(new pcl::PointCloud<pcl::Normal>())
    , previous_icp_result_(Eigen::Isometry3d::Identity())
    , T_relocalize_(Eigen::Isometry3d::Identity())
    , register_(std::make_shared<small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>())
    , tf_listener_(std::make_unique<tf2_ros::TransformListener>(tf2_buffer_))
    , nh_("relocalization")
    , localize_success_(false)
    , lost_(false)
    , debug_en_(false)
    , diverge_()
    , previous_error_(10000.0)
    , yaw_bias_cnt_(0)
{
    // ========== 加载参数 ==========
    nh_.param<std::string>("pcd_path", pcd_path_, "");
    nh_.param<std::string>("obs_path", obs_path_, "");
    nh_.param<double>("leaf_size", leaf_size_, 0.1);
    nh_.param<int>("freq", freq_, 10);
    nh_.param<int>("max_frame", max_frame_, 10);
    nh_.param<double>("map_leaf_size", map_leaf_size_, 0.5);
    nh_.param<double>("max_dist_sq", max_dist_sq_, 1.0);
    nh_.param<double>("max_iterations", max_iterations_, 100.0);
    nh_.param<int>("num_neighbors", num_neighbors_, 10);
    nh_.param<int>("num_threads", num_threads_, 4);
    nh_.param<bool>("debug_en", debug_en_, false);
    nh_.param<double>("diverge_threshold", diverge_threshold_, 10.0);

    // ========== 配置配准器 ==========
    register_->reduction.num_threads = num_threads_;
    register_->rejector.max_dist_sq = max_dist_sq_;
    register_->optimizer.max_iterations = static_cast<int>(max_iterations_);

    // ========== 初始化订阅器和发布器 ==========
    obs_sub_ = nh_.subscribe("/ground_segmentation/obstacle_cloud", 5, 
                             &Relocalization::obsCallback, this);
    scan_sub_ = nh_.subscribe("/livox/lidar_ros", 5, 
                              &Relocalization::scanCallback, this);
    initial_pose_sub_ = nh_.subscribe("/initialpose", 5, 
                                      &Relocalization::initialPoseCallback, this);
    diverge_sub_ = nh_.subscribe("/diverge", 5, 
                                 &Relocalization::divergeCallback, this);
    odom_sub_ = nh_.subscribe("/body_pose", 5, 
                               &Relocalization::bodyPoseCallback, this);

    target_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("target", 5);
    source_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("source", 5);
    align_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("aligned", 5);
    localize_success_pub_ = nh_.advertise<std_msgs::Bool>("/localize_success", 30);

    // ========== 创建定时器 ==========
    run_timer_ = nh_.createTimer(ros::Duration(1.0 / freq_), 
                                 &Relocalization::timer, this);

    // ========== 加载地图文件 ==========
    if (pcl::io::loadPCDFile(pcd_path_, *prior_map_) == -1) {
        ROS_ERROR("[Relocalization] Failed to load PCD file: %s", pcd_path_.c_str());
        ros::shutdown();
        return;
    }
    ROS_INFO("[Relocalization] Successfully loaded PCD file: %s (%zu points)", 
             pcd_path_.c_str(), prior_map_->size());

    if (pcl::io::loadPCDFile(obs_path_, *prior_obs_) == -1) {
        ROS_ERROR("[Relocalization] Failed to load OBS file: %s", obs_path_.c_str());
        ros::shutdown();
        return;
    }
    ROS_INFO("[Relocalization] Successfully loaded OBS file: %s (%zu points)", 
             obs_path_.c_str(), prior_obs_->size());

    // ========== 预处理地图点云 ==========
    // 对地图进行体素滤波，生成协方差点云用于配准
    target_cov_ = small_gicp::voxelgrid_sampling_omp<
        pcl::PointCloud<pcl::PointXYZ>, 
        pcl::PointCloud<pcl::PointCovariance>>(*prior_map_, map_leaf_size_, num_threads_);
    
    // 对地图和障碍物进行体素滤波，用于粗配准
    filtered_prior_map_ = small_gicp::voxelgrid_sampling_omp(*prior_map_, map_leaf_size_, 8);
    filtered_prior_obs_ = small_gicp::voxelgrid_sampling_omp(*prior_obs_, map_leaf_size_, 8);

    // ========== 估计协方差并构建 KD 树 ==========
    small_gicp::estimate_covariances_omp(*target_cov_, num_neighbors_, num_threads_);
    target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
        target_cov_, small_gicp::KdTreeBuilderOMP(num_threads_));

    // ========== 初始化变换矩阵 ==========
    T_odom_lidar_ = Eigen::Isometry3d::Identity();
    T_odom_lidar_.translation().z() = 0.0;  // pointlio 生成的点云在 imu 坐标系下

    ROS_INFO("[Relocalization] Initialization completed");
}

void Relocalization::timer(const ros::TimerEvent& event)
{
    // 设置 TF 变换的帧 ID
    T_vice_map_odom_.header.frame_id = "vice_map";
    T_vice_map_odom_.child_frame_id = "odom";

    // 检查是否有数据可用
    if (obs_queue_.empty() || scan_queue_.empty()) {
        // 即使没有新数据，也发布定位成功标志
        std_msgs::Bool success_msg;
        success_msg.data = localize_success_;
        localize_success_pub_.publish(success_msg);
        
        // 发布调试点云（如果启用）
        if (debug_en_) {
            publishDebugClouds();
        }
        return;
    }

    // 如果定位失败或检测到发散，执行配准
    if (!localize_success_ || diverge_.data) {
        // ========== 从队列中获取点云数据 ==========
        {
            std::lock_guard<std::mutex> lock(obs_mutex_);
            obs_->clear();
            while (!obs_queue_.empty()) {
                *obs_ += obs_queue_.front();
                obs_queue_.pop();
            }
        }

        {
            std::lock_guard<std::mutex> lock(scan_mutex_);
            scan_->clear();
            while (!scan_queue_.empty()) {
                *scan_ += scan_queue_.front();
                scan_queue_.pop();
            }
            // 将扫描点云转换到 odom 坐标系
            Eigen::Affine3d T_odom_lidar_affine(T_odom_lidar_);
            pcl::transformPointCloud(*scan_, *scan_odom_, T_odom_lidar_affine);
        }

        // ========== 执行配准 ==========
        registration(scan_odom_, obs_, leaf_size_, diverge_threshold_);

        // ========== 发布 TF 变换 ==========
        publishTransform(T_map_scan_);
    }

    // ========== 发布定位成功标志 ==========
    std_msgs::Bool success_msg;
    success_msg.data = localize_success_;
    localize_success_pub_.publish(success_msg);

    // ========== 发布调试点云（如果启用） ==========
    if (debug_en_) {
        publishDebugClouds();
    }
}

void Relocalization::registration(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
                                  const pcl::PointCloud<pcl::PointXYZ>::Ptr& obs,
                                  double leaf_size,
                                  double threshold)
{
    auto start_time = std::chrono::high_resolution_clock::now();

    // ========== 点云预处理 ==========
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_scan_odom(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_obs(new pcl::PointCloud<pcl::PointXYZ>());

    // 对障碍物点云进行统计离群点去除
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(obs);
    sor.setMeanK(20);
    sor.setStddevMulThresh(0.02);
    sor.filter(*filtered_obs);

    // 体素滤波
    filtered_obs = small_gicp::voxelgrid_sampling_omp(*filtered_obs, leaf_size, 8);
    filtered_scan_odom = small_gicp::voxelgrid_sampling_omp(*source, leaf_size, 8);

    // 生成源点云的协方差点云
    source_cov_ = small_gicp::voxelgrid_sampling_omp<
        pcl::PointCloud<pcl::PointXYZ>, 
        pcl::PointCloud<pcl::PointCovariance>>(*source, leaf_size, num_threads_);

    // ========== 估计协方差并构建 KD 树 ==========
    small_gicp::estimate_covariances_omp(*source_cov_, num_neighbors_, num_threads_);
    source_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
        source_cov_, small_gicp::KdTreeBuilderOMP(num_threads_));

    auto mid_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> duration_mid = mid_time - start_time;
    ROS_DEBUG("[Relocalization] Downsampling took %.2f ms", duration_mid.count());

    // ========== 执行配准 ==========
    small_gicp::RegistrationResult result;
    
    if (lost_) {
        // 如果丢失定位，使用 FPFH 特征进行粗配准
        sacIaCompute(filtered_obs, filtered_prior_obs_);
        result = register_->align(*target_cov_, *source_cov_, *target_tree_, fpfh_result_);
        previous_error_ = 100000.0;
        lost_ = false;
    } else {
        // 使用上一次的配准结果作为初始值
        result = register_->align(*target_cov_, *source_cov_, *target_tree_, previous_icp_result_);
    }

    // ========== 更新配准结果 ==========
    T_map_scan_ = result.T_target_source;
    previous_icp_result_ = result.T_target_source;

    // ========== 判断配准是否成功 ==========
    if (result.error < threshold) {
        localize_success_ = true;
        previous_error_ = 10000.0;
        yaw_bias_cnt_ = 0;
        lost_ = false;
    } else {
        localize_success_ = false;
        
        // 如果误差没有改善或改善很小，尝试调整初始位姿
        if (result.error >= previous_error_ || (previous_error_ - result.error < 1.0)) {
            previous_error_ = 100000.0;
            
            // 计算偏航角偏差（交替正负方向）
            double yaw_bias;
            if (yaw_bias_cnt_ % 2 == 0) {
                yaw_bias = (yaw_bias_cnt_ / 2 + 1) * 0.349;  // 约 20 度
            } else {
                yaw_bias = -(yaw_bias_cnt_ / 2 + 1) * 0.349;
            }
            
            ROS_DEBUG("[Relocalization] Applying yaw bias: %.3f rad", yaw_bias);
            
            // 计算旋转轴（考虑俯仰角）
            double angle_x = -20.0 * M_PI / 180.0;
            Eigen::AngleAxisd rotate_x(angle_x, Eigen::Vector3d::UnitY());
            Eigen::Vector3d new_axis = rotate_x * Eigen::Vector3d::UnitZ();
            
            // 应用偏航角旋转
            Eigen::AngleAxis yaw(yaw_bias, new_axis);
            T_relocalize_.rotate(yaw.toRotationMatrix());
            previous_icp_result_ = T_relocalize_;
            yaw_bias_cnt_++;
        } else {
            previous_error_ = result.error;
        }
    }

    // ========== 生成配准后的点云（用于调试） ==========
    if (debug_en_) {
        Eigen::Affine3d T_map_scan_affine(T_map_scan_);
        pcl::transformPointCloud(*filtered_scan_odom, *aligned_, T_map_scan_affine);
    }

    // ========== 输出配准信息 ==========
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> duration = end_time - start_time;
    ROS_DEBUG("[Relocalization] Registration error: %.3f, success: %s, time: %.2f ms",
              result.error, localize_success_ ? "true" : "false", duration.count());
}

void Relocalization::computeFpfhFeature(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                                        pcl::search::KdTree<pcl::PointXYZ>::Ptr& tree,
                                        pcl::PointCloud<pcl::FPFHSignature33>::Ptr& fpfh)
{
    // ========== 计算法向量 ==========
    point_normal_->clear();
    est_normal_.setInputCloud(input_cloud);
    est_normal_.setSearchMethod(tree);
    est_normal_.setRadiusSearch(0.5);
    est_normal_.compute(*point_normal_);

    // ========== 计算 FPFH 特征 ==========
    est_fpfh_.setNumberOfThreads(4);
    est_fpfh_.setRadiusSearch(1.0);
    est_fpfh_.setInputCloud(input_cloud);
    est_fpfh_.setInputNormals(point_normal_);
    est_fpfh_.setSearchMethod(tree);
    est_fpfh_.compute(*fpfh);

    ROS_DEBUG("[Relocalization] FPFH feature size: %zu", fpfh->size());
    
    if (fpfh->empty()) {
        ROS_ERROR("[Relocalization] FPFH feature computation failed!");
    }
}

pcl::PointCloud<pcl::PointXYZ> Relocalization::sacIaCompute(
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud)
{
    auto start_time = std::chrono::high_resolution_clock::now();

    // ========== 构建 KD 树 ==========
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

    // ========== 计算 FPFH 特征 ==========
    computeFpfhFeature(source_cloud, tree, source_fpfh_);
    computeFpfhFeature(target_cloud, tree, target_fpfh_);

    // ========== 配置 SAC-IA 配准器 ==========
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
    sac_ia.setInputSource(source_cloud);
    sac_ia.setSourceFeatures(source_fpfh_);
    sac_ia.setInputTarget(target_cloud);
    sac_ia.setTargetFeatures(target_fpfh_);
    sac_ia.setNumberOfSamples(30);
    sac_ia.setMinSampleDistance(1.0);
    sac_ia.setCorrespondenceRandomness(20);
    sac_ia.setEuclideanFitnessEpsilon(0.00001);
    sac_ia.setTransformationEpsilon(1e-8);
    sac_ia.setRANSACIterations(500);
    sac_ia.setMaximumIterations(2000);

    // ========== 执行配准 ==========
    pcl::PointCloud<pcl::PointXYZ> result;
    sac_ia.align(result);

    if (sac_ia.hasConverged()) {
        fpfh_result_ = sac_ia.getFinalTransformation().cast<double>();
        if (debug_en_) {
            ROS_INFO("[Relocalization] FPFH-based RANSAC converged");
            const Eigen::IOFormat fmt(3, 0, ", ", "\n", "[", "]");
            std::ostringstream oss;
            oss << fpfh_result_.matrix().format(fmt);
            ROS_DEBUG("[Relocalization] Approximate transform:\n%s", oss.str().c_str());
        }
    } else {
        ROS_ERROR("[Relocalization] FPFH-based RANSAC failed to converge!");
        fpfh_result_ = Eigen::Isometry3d::Identity();
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> duration = end_time - start_time;
    ROS_DEBUG("[Relocalization] SAC-IA computation took %.2f ms", duration.count());

    return result;
}

void Relocalization::publishDebugClouds()
{
    // 发布目标点云（先验地图）
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*filtered_prior_map_, msg);
    msg.header.frame_id = "vice_map";
    msg.header.stamp = ros::Time::now();
    target_pub_.publish(msg);

    // 发布源点云（当前扫描）
    pcl::toROSMsg(*scan_odom_, msg);
    msg.header.frame_id = "vice_map";
    msg.header.stamp = ros::Time::now();
    source_pub_.publish(msg);

    // 发布配准后的点云
    pcl::toROSMsg(*aligned_, msg);
    msg.header.frame_id = "vice_map";
    msg.header.stamp = ros::Time::now();
    align_pub_.publish(msg);
}

void Relocalization::publishTransform(const Eigen::Isometry3d& transform)
{
    // 将变换矩阵转换为四元数和平移向量
    Eigen::Quaterniond q(transform.linear());
    Eigen::Vector3d translation = transform.translation();

    // 设置 TF 变换
    T_vice_map_odom_.header.stamp = ros::Time::now();
    T_vice_map_odom_.transform.rotation.x = q.x();
    T_vice_map_odom_.transform.rotation.y = q.y();
    T_vice_map_odom_.transform.rotation.z = q.z();
    T_vice_map_odom_.transform.rotation.w = q.w();
    T_vice_map_odom_.transform.translation.x = translation.x();
    T_vice_map_odom_.transform.translation.y = translation.y();
    T_vice_map_odom_.transform.translation.z = translation.z();

    // 发布 TF 变换
    tf_broadcaster_.sendTransform(T_vice_map_odom_);

    // 输出变换信息（调试用）
    ROS_DEBUG("[Relocalization] Published transform: t=[%.3f, %.3f, %.3f], q=[%.3f, %.3f, %.3f, %.3f]",
              translation.x(), translation.y(), translation.z(),
              q.x(), q.y(), q.z(), q.w());
}

void Relocalization::obsCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ> temp_cloud;
    pcl::fromROSMsg(*msg, temp_cloud);

    std::lock_guard<std::mutex> lock(obs_mutex_);
    obs_queue_.push(temp_cloud);
    
    // 限制队列大小
    if (obs_queue_.size() > static_cast<size_t>(max_frame_)) {
        obs_queue_.pop();
    }
}

void Relocalization::scanCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ> temp_cloud;
    pcl::fromROSMsg(*msg, temp_cloud);

    std::lock_guard<std::mutex> lock(scan_mutex_);
    scan_queue_.push(temp_cloud);
    
    // 限制队列大小
    if (scan_queue_.size() > static_cast<size_t>(max_frame_)) {
        scan_queue_.pop();
    }
}

void Relocalization::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    // ========== 从消息中提取初始位姿 ==========
    Eigen::Isometry3d T_map_baselink_estimate = Eigen::Isometry3d::Identity();
    T_map_baselink_estimate.translation() << msg->pose.pose.position.x,
                                             msg->pose.pose.position.y,
                                             msg->pose.pose.position.z;
    
    Eigen::Quaterniond quat(msg->pose.pose.orientation.w,
                           msg->pose.pose.orientation.x,
                           msg->pose.pose.orientation.y,
                           msg->pose.pose.orientation.z);
    T_map_baselink_estimate.linear() = quat.toRotationMatrix();

    // ========== 获取 base_link 到 odom 的变换 ==========
    try {
        auto transform = tf2_buffer_.lookupTransform("base_link", "odom", ros::Time(0));
        Eigen::Isometry3d T_baselink_odom = tf2::transformToEigen(transform.transform);
        
        // 计算 vice_map 到 odom 的变换
        Eigen::Isometry3d T_vice_map_odom = T_map_baselink_estimate * T_baselink_odom;
        
        // 更新配准初始值
        previous_icp_result_ = T_vice_map_odom;
        previous_error_ = 100000.0;
        localize_success_ = false;
        lost_ = false;
        
        ROS_INFO("[Relocalization] Initial pose set: t=[%.3f, %.3f, %.3f]",
                 T_vice_map_odom.translation().x(),
                 T_vice_map_odom.translation().y(),
                 T_vice_map_odom.translation().z());
    } catch (tf2::TransformException& ex) {
        ROS_WARN("[Relocalization] Could not transform initial pose: %s", ex.what());
    }
}

void Relocalization::divergeCallback(const std_msgs::Bool::ConstPtr& msg)
{
    diverge_.data = msg->data;
    
    // 如果检测到发散，重置定位成功标志
    if (diverge_.data) {
        localize_success_ = false;
    }
    
    // 发布定位成功标志
    std_msgs::Bool success_msg;
    success_msg.data = localize_success_;
    localize_success_pub_.publish(success_msg);
}

void Relocalization::bodyPoseCallback(const geometry_msgs::PoseStamped& msg)
{
    // 如果定位成功，使用 body_pose 更新配准初始值
    if (localize_success_) {
        previous_icp_result_.translation() << msg.pose.position.x,
                                             msg.pose.position.y,
                                             msg.pose.position.z;
        
        Eigen::Quaterniond quat(msg.pose.orientation.w,
                                msg.pose.orientation.x,
                                msg.pose.orientation.y,
                                msg.pose.orientation.z);
        previous_icp_result_.linear() = quat.toRotationMatrix();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "relocalization");
    Relocalization relocalization;
    ros::spin();
    return 0;
}
