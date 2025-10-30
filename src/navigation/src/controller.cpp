#include "navigation/controller.hpp"

// ========== 常量定义 ==========
namespace
{
    // 状态标志
    const int PLANNER_FAILED_STATUS = 4;
    const int LOCALIZATION_WAIT_TIME_MS = 5000;
    
    // 目标到达判断
    const double EXTRA_TOLERANCE = 0.2;  // 额外容忍度（米）
    const double NEAR_GOAL_DISTANCE = 1.0;  // 接近目标距离（米）
    
    // 航向误差阈值
    const double LARGE_YAW_ERROR = M_PI / 2;      // 90度
    const double MEDIUM_YAW_ERROR = M_PI / 5;     // 36度
    const double SMALL_YAW_ERROR = M_PI / 6;      // 30度
    const double TINY_YAW_ERROR = M_PI / 18;      // 10度
    const double MIN_DISTANCE_FOR_TURN = 1.0;     // 允许转向的最小距离
    
    // 路径优化
    const double DISTANCE_RATIO_COEFF = 0.08;
    const double MIN_POINT_DISTANCE = 0.05;        // 最小点间距离（米）
    const double HIGH_CURVATURE_THRESHOLD = 0.7;
    const int FORESEE_POSE_INDEX = 10;
    const int YAW_CONTROL_TARGET_INDEX = 10;
    
    // 路径处理
    const double DISTANCE_THRESHOLD = 10.0;        // 距离阈值（米）
    const double LARGE_DISTANCE = 1e10;
}

/**
 * @brief Controller构造函数
 * 初始化所有ROS参数、发布器、订阅器和定时器
 */
Controller::Controller()
{
    ros::NodeHandle nh("~");
    
    loadParameters(nh);
    initializePublishers(nh);
    initializeSubscribers(nh);
    initializeTimers(nh);
    
    // 初始化路径优化器
    optimizer_.setparam(param_);
    prune_path.header.frame_id = global_frame;
    opt_path.header.frame_id = global_frame;
    
    // 初始化状态变量
    prune_index = 0;
    follow_index = 0;
    
    ROS_INFO("Controller initialized with max_speed = %.2f", max_speed);
}

/**
 * @brief 加载所有ROS参数
 */
void Controller::loadParameters(ros::NodeHandle& nh)
{
    // 运动控制参数
    nh.param<double>("max_speed", max_speed, 1.0);
    nh.param<double>("straight_p_value", straight_p_value, 2);
    nh.param<double>("curve_p_value", curve_p_value, 1);
    nh.param<double>("set_yaw_speed", set_yaw_speed, 0);
    
    // 规划参数
    nh.param<int>("plan_frequency", plan_freq, 30);
    nh.param<int>("opt_freq", opt_freq, 3);
    nh.param<double>("goal_dist_tolerance", goal_dist_tolerance, 0.2);
    nh.param<double>("prune_ahead_distance", prune_ahead_dist, 0.5);
    nh.param<std::string>("global_frame", global_frame, "map");
    
    // 前视距离参数
    nh.param<int>("straight_foresee_index", straight_foresee_index, 1);
    nh.param<int>("curve_foresee_index", curve_foresee_index, 1);
    nh.param<int>("forsee_index", forsee_index, 50);
    nh.param<int>("short_forsee_index", short_forsee_index, 10);
    
    // 障碍物检测参数
    nh.param<int>("narrow_threshold", narrow_threshold, 5);
    nh.param<double>("search_radius", search_radius, 0.3);
    
    // 航向控制参数
    nh.param<double>("wz_p_value", wz_p_value, 1);
    nh.param<double>("wz_d_value", wz_d_value, 0.01);
    
    // 路径优化器参数
    nh.param<double>("param_convergence_thresh", param_.convergence_thresh, 0.01);
    nh.param<int>("param_max_iterations", param_.max_iterations, 100);
    nh.param<double>("param_max_step", param_.max_step, 0.1);
    nh.param<double>("param_repulsive_gain", param_.repulsive_gain, 0.02);
    nh.param<double>("param_attractive_gain", param_.attractive_gain, 0.02);
    nh.param<double>("param_safe_distance", param_.safe_distance, 0.5);
    
    // 恢复行为参数
    nh.param<double>("min_recover_radius", min_recover_radius, 0.1);
    nh.param<double>("max_recover_radius", max_recover_radius, 0.4);
    
    // 调试和模式参数
    nh.param<bool>("debug_en", debug_en, false);
    nh.param<bool>("hole_mode", hole_mode, false);
}

/**
 * @brief 初始化ROS发布器
 */
void Controller::initializePublishers(ros::NodeHandle& nh)
{
    prune_path_pub = nh.advertise<nav_msgs::Path>("prune_path", 5);
    narrow_pub = nh.advertise<std_msgs::Bool>("narrow", 5);
    local_path_pub = nh.advertise<nav_msgs::Path>("local_path", 5);
    forsee_path_pub = nh.advertise<nav_msgs::Path>("forsee_path", 5);
    obstacle_pub = nh.advertise<nav_msgs::GridCells>("obstacle_grids", 5);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
}

/**
 * @brief 初始化ROS订阅器
 */
void Controller::initializeSubscribers(ros::NodeHandle& nh)
{
    global_path_sub = nh.subscribe("/move_base/GlobalPlanner/plan", 5,
                                    &Controller::GlobalPathCallback, this);
    match_sub = nh.subscribe("/match", 5, &Controller::MatchCallback, this);
    costmap_sub = nh.subscribe("/move_base/global_costmap/costmap", 1,
                                &Controller::CostmapCallback, this);
    localize_success_sub = nh.subscribe("/localize_success", 5,
                                         &Controller::LocalizeCallback, this);
    global_status_sub = nh.subscribe("/move_base/status", 5,
                                      &Controller::StatusCallback, this);
}

/**
 * @brief 初始化ROS定时器
 */
void Controller::initializeTimers(ros::NodeHandle& nh)
{
    tf_listener = std::make_shared<tf::TransformListener>();
    plan_timer = nh.createTimer(ros::Duration(1.0 / plan_freq),
                                 &Controller::Plan, this);
    optimize_timer = nh.createTimer(ros::Duration(1.0 / opt_freq),
                                     &Controller::PathOptimaze, this);
}

/**
 * @brief 主规划控制循环
 * @param event 定时器事件
 * 
 * 该函数在每个规划周期被调用，协调各个控制模块
 */
void Controller::Plan(const ros::TimerEvent& event)
{
    if (!plan)
    {
        // 无规划时，根据定位状态发布停止命令
        if (localized && isLocalizationReady())
        {
            publishStopCommand(1);  // 已定位但无路径
        }
        else
        {
            publishStopCommand(0);  // 未定位或定位失败
        }
        return;
    }

    // 获取机器人当前位置
    geometry_msgs::PoseStamped robot_pose;
    GetTargetRobotPose(tf_listener, global_path.header.frame_id, robot_pose);
    double distance_to_goal = GetEuclideanDistance(robot_pose, global_path.poses.back());

    // 检查是否到达目标
    if (isGoalReached(robot_pose, distance_to_goal))
    {
        handleGoalReached();
        return;
    }

    // 检查是否能够执行路径
    if (!canExecutePath())
    {
        if (!isLocalizationReady())
        {
            handleWaitingForLocalization();
        }
        else
        {
            publishStopCommand(1);  // 已定位但无有效路径
        }
        return;
    }

    // 提取跟随路径并计算曲率
    nav_msgs::Path following_path = extractFollowingPath(robot_pose);
    if (following_path.poses.empty())
    {
        publishStopCommand(0);
        return;
    }

    curvature = CurvatureCal(following_path);
    x_forward = determineForwardDirection();

    // 检查是否需要原地转向
    int target_index = std::min(static_cast<int>(following_path.poses.size()) - 1, 
                                FORESEE_POSE_INDEX);
    double yaw_error = YawErrorCal(robot_pose, following_path.poses[target_index], x_forward);

    if (shouldTurnInPlace(yaw_error, distance_to_goal))
    {
        if (isTurnComplete(yaw_error))
        {
            turn_state = false;
            return;
        }
        handleTurningInPlace(yaw_error);
        return;
    }

    // 执行路径跟踪
    arrive_state = false;
    handlePathTracking(robot_pose, following_path, distance_to_goal);
}

/**
 * @brief 检查是否到达目标点
 */
bool Controller::isGoalReached(const geometry_msgs::PoseStamped& robot_pose, 
                                double distance_to_goal) const
{
    bool distance_reached = distance_to_goal <= goal_dist_tolerance;
    bool path_complete = prune_index == static_cast<int>(global_path.poses.size()) - 1;
    bool extra_tolerance_reached = (distance_to_goal <= goal_dist_tolerance + EXTRA_TOLERANCE) && arrive_state;
    
    return distance_reached || path_complete || extra_tolerance_reached;
}

/**
 * @brief 检查定位是否就绪（定位成功且稳定）
 */
bool Controller::isLocalizationReady() const
{
    if (!localized)
    {
        return false;
    }
    
    auto current_time = std::chrono::high_resolution_clock::now();
    auto time_since_localized = current_time - localized_time;
    
    return time_since_localized > std::chrono::milliseconds(LOCALIZATION_WAIT_TIME_MS);
}

/**
 * @brief 检查是否可以执行路径
 */
bool Controller::canExecutePath() const
{
    bool has_paths = !prune_path.poses.empty() && !opt_path.poses.empty();
    bool planner_ok = global_planner_status != PLANNER_FAILED_STATUS;
    
    return plan && has_paths && planner_ok;
}

/**
 * @brief 判断是否需要原地转向
 */
bool Controller::shouldTurnInPlace(double yaw_error, double distance_to_goal) const
{
    bool large_error = std::abs(yaw_error) > LARGE_YAW_ERROR;
    bool medium_error_narrow = (std::abs(yaw_error) > MEDIUM_YAW_ERROR) && narrow;
    bool in_turn_state = turn_state;
    bool far_enough = distance_to_goal > MIN_DISTANCE_FOR_TURN;
    
    return (large_error || medium_error_narrow || in_turn_state) && far_enough;
}

/**
 * @brief 检查转向是否完成
 */
bool Controller::isTurnComplete(double yaw_error) const
{
    if (narrow)
    {
        return std::abs(yaw_error) < TINY_YAW_ERROR;
    }
    else
    {
        return std::abs(yaw_error) < SMALL_YAW_ERROR;
    }
}

/**
 * @brief 提取从当前跟随点开始的优化路径
 */
nav_msgs::Path Controller::extractFollowingPath(const geometry_msgs::PoseStamped& robot_pose)
{
    nav_msgs::Path following_path;
    following_path.header.frame_id = opt_path.header.frame_id;
    
    {
        std::lock_guard<std::mutex> lock(optpath_mutex);
        // 创建非const副本以传递给FindNearstPose
        geometry_msgs::PoseStamped robot_pose_nonconst = robot_pose;
        FindNearstPose(robot_pose_nonconst, opt_path, follow_index, prune_ahead_dist);
        
        for (int i = follow_index; i < static_cast<int>(opt_path.poses.size()); i++)
        {
            following_path.poses.push_back(opt_path.poses[i]);
        }
    }
    
    return following_path;
}

/**
 * @brief 处理到达目标的情况
 */
void Controller::handleGoalReached()
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 1;  // 标识：规划成功
    cmd_vel.angular.z = set_yaw_speed;
    cmd_vel_pub.publish(cmd_vel);
    
    ROS_INFO("Planning Success! Goal reached.");
    
    prune_index = 0;
    follow_index = 0;
    arrive_state = true;
}

/**
 * @brief 处理等待定位的情况
 */
void Controller::handleWaitingForLocalization()
{
    publishStopCommand(0);
    ROS_WARN("STOP!!! Waiting for localization...");
}

/**
 * @brief 处理原地转向
 */
void Controller::handleTurningInPlace(double yaw_error)
{
    turn_state = true;
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;
    cmd_vel.angular.z = YawControl(yaw_error);
    cmd_vel_pub.publish(cmd_vel);
    
    ROS_INFO("Turning attitude! Yaw error = %.3f rad", yaw_error);
}

/**
 * @brief 处理路径跟踪
 */
void Controller::handlePathTracking(const geometry_msgs::PoseStamped& robot_pose,
                                    const nav_msgs::Path& path,
                                    double distance_to_goal)
{
    geometry_msgs::Twist cmd_vel;
    FollowTraj(robot_pose, path, cmd_vel);
    applySpeedScalingNearGoal(cmd_vel, distance_to_goal);
    cmd_vel_pub.publish(cmd_vel);
}

/**
 * @brief 在接近目标时应用速度缩放
 */
void Controller::applySpeedScalingNearGoal(geometry_msgs::Twist& cmd_vel, 
                                           double distance_to_goal)
{
    if (distance_to_goal < NEAR_GOAL_DISTANCE)
    {
        double speed_scale = distance_to_goal;
        cmd_vel.linear.x *= speed_scale;
        cmd_vel.linear.y *= speed_scale;
        cmd_vel.angular.z *= speed_scale;
    }
}

/**
 * @brief 发布停止命令
 * @param status_flag 状态标志：0=未定位/失败，1=已定位但无路径
 */
void Controller::publishStopCommand(int status_flag)
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.angular.z = 0;
    cmd_vel.linear.z = status_flag;
    cmd_vel_pub.publish(cmd_vel);
    
    if (status_flag == 1)
    {
        ROS_INFO("////////////////NO PATH/////////////////////");
    }
}

/**
 * @brief 确定前进方向
 */
bool Controller::determineForwardDirection() const
{
    // 在狭窄通道和hole模式下，前进方向不是x轴正方向
    return !(narrow && hole_mode);
}
/**
 * @brief 路径优化定时器回调函数
 * @param event 定时器事件
 * 
 * 协调路径优化流程：剪枝、障碍物检测、路径点优化
 */
void Controller::PathOptimaze(const ros::TimerEvent& event)
{
    if (!plan || !localized || global_planner_status == PLANNER_FAILED_STATUS)
    {
        return;
    }

    geometry_msgs::PoseStamped robot_pose;
    GetTargetRobotPose(tf_listener, global_path.header.frame_id, robot_pose);
    
    // 更新全局路径索引
    FindNearstPose(robot_pose, global_path, prune_index, prune_ahead_dist);

    // 提取可通行路径（剪枝）
    nav_msgs::Path temp_prune_path = extractPrunePathFromGlobal(robot_pose);
    
    // 更新剪枝路径并发布
    {
        std::lock_guard<std::mutex> lock(prunepath_mutex);
        prune_path = temp_prune_path;
    }
    prune_path_pub.publish(prune_path);
    
    // 检测狭窄通道
    detectNarrowPassage(prune_path);
    publishObstaclesGrid(obstacle_result, costmap.info.resolution, obstacle_pub);
    
    // 优化路径点
    std::vector<geometry_msgs::PoseStamped> temp_opt_path = 
        optimizePathPoints(temp_prune_path.poses, robot_pose);
    
    // 移除过于接近的点
    removeClosePathPoints(temp_opt_path, MIN_POINT_DISTANCE);
    
    // 更新优化路径并发布
    {
        std::lock_guard<std::mutex> lock(optpath_mutex);
        opt_path.poses.swap(temp_opt_path);
    }
    
    narrow_msg.data = false;
    narrow_pub.publish(narrow_msg);
    local_path_pub.publish(opt_path);
    follow_index = 0;
}

/**
 * @brief 从全局路径提取可通行路径（剪枝）
 */
nav_msgs::Path Controller::extractPrunePathFromGlobal(const geometry_msgs::PoseStamped& robot_pose)
{
    std::vector<geometry_msgs::PoseStamped> prune_poses;
    const int FREE_SPACE_COST = 0;  // 可通行代价值
    
    int path_index = prune_index;
    while (path_index < static_cast<int>(global_path.poses.size()) && 
           path_index - prune_index < forsee_index)
    {
        int grid_x, grid_y;
        const auto& pose = global_path.poses[path_index];
        
        if (NavigationUtils::world2Grid(pose.pose.position.x,
                                        pose.pose.position.y,
                                        costmap, grid_x, grid_y))
        {
            int costmap_index = grid_y * costmap.info.width + grid_x;
            
            // 只添加可通行的路径点
            if (costmap.data[costmap_index] == FREE_SPACE_COST)
            {
                prune_poses.push_back(pose);
            }
        }
        path_index++;
    }
    
    nav_msgs::Path prune_path;
    prune_path.header.frame_id = global_frame;
    prune_path.poses = prune_poses;
    
    return prune_path;
}

/**
 * @brief 优化路径点
 */
std::vector<geometry_msgs::PoseStamped> Controller::optimizePathPoints(
    const std::vector<geometry_msgs::PoseStamped>& prune_path,
    const geometry_msgs::PoseStamped& robot_pose)
{
    std::vector<geometry_msgs::PoseStamped> optimized_path;
    int global_path_size = static_cast<int>(global_path.poses.size());
    
    // 判断是否应该优化（在狭窄环境下跳过以节省时间）
    bool should_optimize = (!narrow && hole_mode) || !hole_mode;
    
    for (size_t i = 0; i < prune_path.size(); i++)
    {
        const auto& pose = prune_path[i];
        geometry_msgs::PoseStamped optimized_pose;
        
        // 计算优化强度比例：路径中间的点优化强度大，两端的小
        double distance_from_center = static_cast<double>(
            global_path_size / 2 - std::abs(global_path_size / 2 - (static_cast<int>(i) + prune_index)));
        double optimization_ratio = 1 - exp(-distance_from_center * DISTANCE_RATIO_COEFF);
        
        if (should_optimize && optimizer_.optimize(obstacle_result, pose, optimized_pose, optimization_ratio))
        {
            optimized_path.push_back(optimized_pose);
        }
        else
        {
            optimized_path.push_back(pose);
            if (should_optimize)
            {
                ROS_WARN("Path optimization failed for pose %zu", i);
            }
        }
        
        // 只优化前short_forsee_index个点
        if (static_cast<int>(i) >= short_forsee_index)
        {
            break;
        }
    }
    
    return optimized_path;
}

/**
 * @brief 移除过于接近的路径点
 */
void Controller::removeClosePathPoints(std::vector<geometry_msgs::PoseStamped>& path, 
                                      double min_distance)
{
    for (size_t i = 0; i < path.size() - 1; i++)
    {
        while (i + 1 < path.size() &&
               GetEuclideanDistance(path[i], path[i + 1]) < min_distance)
        {
            path.erase(path.begin() + i + 1);
        }
    }
}

/**
 * @brief 检测狭窄通道
 */
void Controller::detectNarrowPassage(const nav_msgs::Path& path)
{
    int obstacle_count = Passbility_check(costmap, 
                                          const_cast<nav_msgs::Path&>(path),
                                          search_radius, 
                                          obstacle_result, 
                                          static_cast<int>(path.poses.size()));
    updateNarrowPassageStatus(obstacle_count);
}

/**
 * @brief 更新狭窄通道状态
 */
void Controller::updateNarrowPassageStatus(int obstacle_count)
{
    if (obstacle_count > narrow_threshold)
    {
        narrow = true;
        ROS_INFO("NARROW passage detected!");
    }
    else if ((!narrow && obstacle_count < narrow_threshold) || 
             (narrow && obstacle_count < 1))
    {
        narrow = false;
        ROS_INFO("NOT NARROW passage");
    }
}
/**
 * @brief 计算航向误差角
 * @param robot_pose 机器人当前位置
 * @param path_pose 目标路径点位置
 * @param x_forward 前进方向是否为x轴正方向（false表示y轴方向，如hole模式）
 * @return 航向误差角（弧度），范围[-π, π]
 * 
 * 计算机器人当前航向与指向目标路径点的航向之间的差值
 */
double Controller::YawErrorCal(const geometry_msgs::PoseStamped& robot_pose,
                                const geometry_msgs::PoseStamped& path_pose,
                                bool x_forward)
{
    double robot_attitude = NavigationUtils::normalizeRadian(tf2::getYaw(robot_pose.pose.orientation));
    
    // 计算目标点在机器人坐标系中的位置
    double dx = path_pose.pose.position.x - robot_pose.pose.position.x;
    double dy = path_pose.pose.position.y - robot_pose.pose.position.y;
    double path_attitude;
    
    if (x_forward)
    {
        // 正常模式：前进方向为x轴正方向
        path_attitude = atan2(dy, dx);
    }
    else
    {
        // Hole模式：前进方向为y轴方向（允许横向移动）
        // 计算两个可能的朝向（±90度）并选择与机器人当前朝向更接近的
        double candidate1 = atan2(dy, dx) - M_PI_2;
        double candidate2 = atan2(dy, dx) + M_PI_2;
        double error1 = std::abs(NavigationUtils::normalizeRadian(candidate1 - robot_attitude));
        double error2 = std::abs(NavigationUtils::normalizeRadian(candidate2 - robot_attitude));
        
        const double DIRECTION_THRESHOLD = 0.7;  // 方向选择阈值
        if (error1 - error2 > DIRECTION_THRESHOLD)
        {
            path_attitude = candidate2;
        }
        else
        {
            path_attitude = candidate1;
        }
    }
    
    // 检查异常值
    if (std::isnan(path_attitude))
    {
        ROS_ERROR("path_attitude is NAN! dx=%.3f, dy=%.3f", dx, dy);
        return 0;
    }
    
    return NavigationUtils::normalizeRadian(path_attitude - robot_attitude);
}
/**
 * @brief 全局路径回调函数
 * @param msg 接收到的全局路径消息
 * 
 * 当接收到新的全局路径时，重置路径索引并启用规划
 */
void Controller::GlobalPathCallback(const nav_msgs::PathConstPtr& msg)
{
    if (!msg->poses.empty())
    {
        global_path = *msg;
        prune_index = 0;
        
        // 只有未发散时才启用规划
        if (diverge == false)
        {
            plan = true;
        }
    }
}

/**
 * @brief 匹配状态回调函数
 * @param msg 接收到的匹配状态消息
 * 
 * 当定位匹配失败时（diverge=true）停止规划
 */
void Controller::MatchCallback(const std_msgs::BoolConstPtr& msg)
{
    if (msg->data == 1)
    {
        diverge = false;  // 匹配成功
    }
    else
    {
        diverge = true;   // 匹配失败，停止规划
        plan = false;
    }
}

/**
 * @brief 代价地图回调函数
 * @param msg 接收到的代价地图消息
 * 
 * 更新内部costmap用于路径规划和障碍物检测
 */
void Controller::CostmapCallback(const nav_msgs::OccupancyGridConstPtr& msg)
{
    if (!msg->data.empty())
    {
        costmap = *msg;
    }
}

/**
 * @brief 定位成功回调函数
 * @param msg 接收到的定位状态消息
 * 
 * 记录定位成功的时间点，用于判断定位稳定性
 */
void Controller::LocalizeCallback(const std_msgs::BoolConstPtr& msg)
{
    if (msg->data && !localized)
    {
        // 首次定位成功时记录时间
        localized_time = std::chrono::high_resolution_clock::now();
        ROS_INFO("------------------------------------------------------");
        ROS_INFO("Localization successful!");
    }
    localized = msg->data;
}

/**
 * @brief 全局规划器状态回调函数
 * @param msg 接收到的规划器状态消息
 * 
 * 监控全局规划器的运行状态
 */
void Controller::StatusCallback(const actionlib_msgs::GoalStatusArray& msg)
{
    if (!msg.status_list.empty())
    {
        global_planner_status = msg.status_list.back().status;
        // 状态码：
        // 0: PENDING, 1: ACTIVE, 2: PREEMPTED, 3: SUCCEEDED, 4: ABORTED, 5: REJECTED
    }
}

// ========== NavigationUtils命名空间实现 ==========
namespace NavigationUtils
{
    double normalizeRadian(double angle)
    {
        double n_angle = std::fmod(angle, 2 * M_PI);
        n_angle = n_angle > M_PI ? n_angle - 2 * M_PI : n_angle < -M_PI ? 2 * M_PI + n_angle : n_angle;
        return n_angle;
    }
    
    double ABS_limit(double value, double limit)
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
    
    bool world2Grid(double wx, double wy,
                    const nav_msgs::OccupancyGrid& costmap,
                    int& gx, int& gy)
    {
        if (costmap.info.width == 0 || costmap.info.height == 0)
        {
            return false;
        }

        gx = static_cast<int>((wx - costmap.info.origin.position.x) / costmap.info.resolution);
        gy = static_cast<int>((wy - costmap.info.origin.position.y) / costmap.info.resolution);

        return (gx >= 0 && gx < costmap.info.width && 
                gy >= 0 && gy < costmap.info.height);
    }
    
    bool Grid2world(int gx, int gy,
                    const nav_msgs::OccupancyGrid& costmap,
                    double& wx, double& wy)
    {
        if (costmap.info.width == 0 || costmap.info.height == 0)
        {
            return false;
        }

        wx = (gx + 0.5) * costmap.info.resolution + costmap.info.origin.position.x;
        wy = (gy + 0.5) * costmap.info.resolution + costmap.info.origin.position.y;

        return true;
    }
}
/**
 * @brief 检查路径的可通行性并检测障碍物
 * @param costmap 代价地图
 * @param plan 待检查的路径
 * @param search_radius 搜索半径（米）
 * @param result 输出的障碍物结果列表
 * @param forsee_index 前视索引（只检查前N个路径点）
 * @return 检测到的狭窄通道数量
 * 
 * 在路径周围搜索障碍物，用于判断是否为狭窄通道
 */
int Controller::Passbility_check(nav_msgs::OccupancyGrid& costmap, 
                                  nav_msgs::Path& plan, 
                                  double search_radius,
                                  std::vector<ObstacleResult>& result, 
                                  int forsee_index)
{
    int narrow_count = 0;
    int path_index = 0;
    int grid_radius = static_cast<int>(search_radius / costmap.info.resolution);
    ObstacleResult obstacle;
    result.clear();
    
    for (const auto& pose : plan.poses)
    {
        int grid_x, grid_y;
        // 跳过无法转换的路径点
        if (!NavigationUtils::world2Grid(pose.pose.position.x, pose.pose.position.y, costmap, grid_x, grid_y))
        {
            path_index++;
            continue;
        }
        
        // 初始化包围盒
        int max_x = grid_x, max_y = grid_y;
        int min_x = grid_x, min_y = grid_y;
        
        // 在搜索半径内扫描障碍物
        for (int x = grid_x - grid_radius; x <= grid_x + grid_radius; x++)
        {
            for (int y = grid_y - grid_radius; y <= grid_y + grid_radius; y++)
            {
                // 检查边界
                if (x < 0 || x >= costmap.info.width || 
                    y < 0 || y >= costmap.info.height)
                {
                    continue;
                }
                
                int costmap_index = y * costmap.info.width + x;
                
                // 如果找到障碍物（cost > 0）
                if (costmap.data[costmap_index] > 0)
                {
                    // 记录障碍物信息
                    obstacle.cost = costmap.data[costmap_index];
                    obstacle.grid_x = x;
                    obstacle.grid_y = y;
                    obstacle.world_position.x = costmap.info.origin.position.x + 
                                               (x + 0.5) * costmap.info.resolution;
                    obstacle.world_position.y = costmap.info.origin.position.y + 
                                               (y + 0.5) * costmap.info.resolution;
                    result.push_back(obstacle);
                    
                    // 更新包围盒
                    if (x > max_x) max_x = x;
                    if (x < min_x) min_x = x;
                    if (y > max_y) max_y = y;
                    if (y < min_y) min_y = y;
                }
            }
        }
        
        // 判断是否为狭窄点：路径点被障碍物包围且在前视范围内
        bool is_narrow_point = (max_x > grid_x && max_y > grid_y && 
                               min_x < grid_x && min_y < grid_y && 
                               path_index < forsee_index);
        if (is_narrow_point)
        {
            narrow_count++;
        }
        
        path_index++;
    }
    
    ROS_DEBUG("Narrow passage check: %d narrow points detected", narrow_count);
    return narrow_count;
}
/**
 * @brief 发布障碍物栅格信息
 * @param obstacles 障碍物结果列表
 * @param resolution 栅格分辨率
 * @param pub ROS发布器
 * 
 * 将检测到的障碍物转换为GridCells消息并发布
 */
void Controller::publishObstaclesGrid(const std::vector<ObstacleResult>& obstacles, 
                                      double resolution, 
                                      ros::Publisher& pub)
{
    nav_msgs::GridCells grid;
    grid.header.frame_id = "map";
    grid.header.stamp = ros::Time::now();
    grid.cell_width = resolution;
    grid.cell_height = resolution;

    for (const auto& obs : obstacles)
    {
        geometry_msgs::Point cell;
        cell.x = obs.world_position.x;
        cell.y = obs.world_position.y;
        cell.z = 0.0;
        grid.cells.push_back(cell);
    }
    
    pub.publish(grid);
}
/**
 * @brief 在路径中找到离机器人最近的路径点索引
 * @param robot_pose 机器人当前位置
 * @param path 待搜索的路径
 * @param prune_index 当前路径索引（输入输出参数）
 * @param prune_ahead_dist 前视距离阈值（米）
 * 
 * 从当前索引开始向前搜索，找到距离机器人最近的路径点
 */
void Controller::FindNearstPose(geometry_msgs::PoseStamped& robot_pose,
                                 nav_msgs::Path& path, 
                                 int& prune_index, 
                                 double prune_ahead_dist)
{
    const double DISTANCE_THRESHOLD = 10.0;  // 距离阈值（米），通常不会超过10米
    const double SQ_DIST_THRESHOLD = DISTANCE_THRESHOLD * DISTANCE_THRESHOLD;
    const double LARGE_DISTANCE = 1e10;      // 初始大距离值
    
    double current_sq_dist;
    
    // 初始化：如果有前一个点，使用前一个点的距离；否则使用大值
    if (prune_index != 0)
    {
        current_sq_dist = GetEuclideanDistance(robot_pose, path.poses[prune_index - 1]);
    }
    else
    {
        current_sq_dist = LARGE_DISTANCE;
    }

    // 向前搜索最近的路径点
    while (prune_index < static_cast<int>(path.poses.size()))
    {
        double new_sq_dist = GetEuclideanDistance(robot_pose, path.poses[prune_index]);
        
        // 如果距离开始增加且之前的距离在阈值内，说明已经找到最近点
        if (new_sq_dist > current_sq_dist && current_sq_dist < SQ_DIST_THRESHOLD)
        {
            // 判断路径点是否在同方向且距离足够远
            // 如果满足条件，回退一个索引以获得更安全的前视点
            double dx_current = path.poses[prune_index].pose.position.x - 
                               robot_pose.pose.position.x;
            double dy_current = path.poses[prune_index].pose.position.y - 
                               robot_pose.pose.position.y;
            double dx_previous = path.poses[prune_index - 1].pose.position.x - 
                                robot_pose.pose.position.x;
            double dy_previous = path.poses[prune_index - 1].pose.position.y - 
                                robot_pose.pose.position.y;
            
            bool same_direction = (dx_current * dx_previous + dy_current * dy_previous > 0);
            bool sufficient_distance = current_sq_dist > prune_ahead_dist * prune_ahead_dist;
            
            if (same_direction && sufficient_distance)
            {
                prune_index--;  // 回退以获得更安全的前视点
            }
            else
            {
                current_sq_dist = new_sq_dist;
            }
            
            break;
        }
        
        current_sq_dist = new_sq_dist;
        ++prune_index;
    }

    // 确保索引不超出范围
    prune_index = std::min(prune_index, static_cast<int>(path.poses.size() - 1));
}
/**
 * @brief 在机器人周围寻找最近的自由空间点
 * @param robot_pose 机器人当前位置
 * @param costmap 代价地图
 * @param min_radius 最小搜索半径（米）
 * @param max_radius 最大搜索半径（米）
 * @return 找到的自由空间点，如果失败则返回机器人当前位置
 * 
 * 从最小半径开始，逐层向外搜索，找到第一个可通行的栅格点
 */
geometry_msgs::PoseStamped Controller::FindNearstFreeSpace(
    geometry_msgs::PoseStamped& robot_pose, 
    nav_msgs::OccupancyGrid& costmap, 
    double min_radius, 
    double max_radius)
{
    int grid_x, grid_y;
    if (!NavigationUtils::world2Grid(robot_pose.pose.position.x, robot_pose.pose.position.y, 
                                     costmap, grid_x, grid_y))
    {
        ROS_ERROR("UNABLE TO TRANSFORM WORLD COORDINATE INTO GRID");
        return robot_pose;
    }
    
    // 将半径转换为栅格单位
    int min_grid_r = static_cast<int>(min_radius / costmap.info.resolution);
    int max_grid_r = static_cast<int>(max_radius / costmap.info.resolution);
    
    // 从内到外逐层搜索
    for (int radius = min_grid_r; radius <= max_grid_r; radius++)
    {
        // 在半径为radius的正方形区域内搜索
        for (int x = grid_x - radius; x <= grid_x + radius; x++)
        {
            for (int y = grid_y - radius; y <= grid_y + radius; y++)
            {
                // 检查边界
                if (x < 0 || x >= costmap.info.width || 
                    y < 0 || y >= costmap.info.height)
                {
                    continue;
                }
                
                int costmap_index = y * costmap.info.width + x;
                
                // 如果找到自由空间（cost = 0）
                if (costmap.data[costmap_index] == 0)
                {
                    double world_x, world_y;
                    NavigationUtils::Grid2world(x, y, costmap, world_x, world_y);
                    
                    geometry_msgs::PoseStamped free_pose;
                    free_pose.header.frame_id = costmap.header.frame_id;
                    free_pose.header.stamp = ros::Time();
                    free_pose.pose.position.x = world_x;
                    free_pose.pose.position.y = world_y;
                    free_pose.pose.position.z = 0;
                    free_pose.pose.orientation = robot_pose.pose.orientation;
                    
                    return free_pose;
                }
            }
        }
    }
    
    ROS_ERROR("Failed to find free space around robot");
    return robot_pose;
}
/**
 * @brief 计算路径曲率
 * @param traj 待计算的路径
 * @return 路径曲率值
 * 
 * 使用路径上的三个点计算曲率，用于判断路径的弯曲程度
 */
double Controller::CurvatureCal(const nav_msgs::Path& traj)
{
    const int MAX_CURVATURE_INDEX = 20;  // 最大曲率计算索引
    const int MIN_CURVATURE_INDEX = 10;   // 最小曲率计算索引
    
    int path_length = static_cast<int>(traj.poses.size());
    int curvature_index = std::min(MAX_CURVATURE_INDEX, path_length - 1);
    
    if (curvature_index > MIN_CURVATURE_INDEX && path_length >= 3)
    {
        // 使用起点、中点和终点计算曲率
        return computeCurvature(traj.poses[0],
                               traj.poses[curvature_index / 2],
                               traj.poses[curvature_index]);
    }
    
    return 0.0;  // 路径太短，曲率为0
}

/**
 * @brief 航向PD控制（基于位姿）
 * @param robot_pose 机器人当前位置
 * @param path_pose 目标路径点位置
 * @param x_forward 前进方向标志
 * @return 角速度指令
 */
double Controller::YawControl(const geometry_msgs::PoseStamped& robot_pose,
                               const geometry_msgs::PoseStamped& path_pose,
                               bool x_forward)
{
    double error = YawErrorCal(robot_pose, path_pose, x_forward);
    return YawControl(error);  // 调用基于误差的重载版本
}

/**
 * @brief 航向PD控制（基于误差）
 * @param error 航向误差（弧度）
 * @return 角速度指令
 * 
 * 使用PD控制器计算角速度：wz = Kp * error + Kd * (error - last_error)
 */
double Controller::YawControl(double error)
{
    double wz = wz_p_value * error + wz_d_value * (error - last_yaw_error);
    last_yaw_error = error;  // 保存本次误差用于下次计算微分项
    return wz;
}
/**
 * @brief 路径跟踪控制函数
 * @param robot_pose 机器人当前位置
 * @param traj 待跟踪的轨迹
 * @param cmd_vel 输出的速度指令
 * 
 * 计算机器人跟随路径所需的速度指令，包括线速度和角速度
 */
void Controller::FollowTraj(const geometry_msgs::PoseStamped& robot_pose,
                            const nav_msgs::Path& traj,
                            geometry_msgs::Twist& cmd_vel)
{
    // 确保轨迹有足够的点
    if (traj.poses.size() < 2)
    {
        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
        cmd_vel.angular.z = 0;
        return;
    }
    
    // 获取机器人当前航向角
    yaw = tf::getYaw(robot_pose.pose.orientation);
    
    // 计算到下一个路径点的距离
    double distance_to_next = GetEuclideanDistance(robot_pose, traj.poses[1]);
    const double MIN_DISTANCE_THRESHOLD = 1e-3;  // 最小距离阈值，避免除以零
    
    if (distance_to_next < MIN_DISTANCE_THRESHOLD)
    {
        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
        cmd_vel.angular.z = 0;
        return;
    }
    
    // ========== 根据路径曲率和环境选择前视点 ==========
    const double HIGH_CURVATURE_THRESHOLD = 0.7;  // 高曲率阈值
    int target_index;
    
    // 高曲率或狭窄环境使用较小的前视距离
    if (curvature > HIGH_CURVATURE_THRESHOLD || narrow)
    {
        p_value = curve_p_value;
        target_index = std::min(curve_foresee_index, 
                               static_cast<int>(traj.poses.size() - 1));
    }
    else
    {
        p_value = straight_p_value;
        target_index = std::min(straight_foresee_index, 
                               static_cast<int>(traj.poses.size() - 1));
    }
    
    // ========== 计算目标方向角 ==========
    double dx = traj.poses[target_index].pose.position.x - robot_pose.pose.position.x;
    double dy = traj.poses[target_index].pose.position.y - robot_pose.pose.position.y;
    double target_yaw = atan2(dy, dx);
    
    // 检查异常值
    if (std::isnan(target_yaw))
    {
        ROS_ERROR("target_yaw is NAN! Distance to target: %.3f", 
                  GetEuclideanDistance(robot_pose, traj.poses[target_index]));
        target_yaw = 0;
    }
    target_yaw = NavigationUtils::normalizeRadian(target_yaw);
    
    // ========== 计算全局坐标系下的速度 ==========
    // 速度大小与距离成正比，方向指向目标点
    double vx_global = cos(target_yaw) * distance_to_next * p_value;
    double vy_global = sin(target_yaw) * distance_to_next * p_value;
    
    // ========== 速度限幅 ==========
    double speed = sqrt(vx_global * vx_global + vy_global * vy_global);
    if (speed > max_speed)
    {
        double speed_ratio = max_speed / speed;
        vx_global *= speed_ratio;
        vy_global *= speed_ratio;
    }
    
    // ========== 计算角速度（航向控制） ==========
    double angular_velocity = 0;
    const int YAW_CONTROL_TARGET_INDEX = 10;  // 用于航向控制的目标点索引
    
    // 在狭窄环境下不进行航向控制，只进行位置跟踪
    if (!narrow && static_cast<int>(traj.poses.size()) > YAW_CONTROL_TARGET_INDEX)
    {
        angular_velocity = YawControl(robot_pose, traj.poses[YAW_CONTROL_TARGET_INDEX], x_forward);
    }
    cmd_vel.angular.z = angular_velocity;
    
    // ========== 旋转补偿 ==========
    // 补偿旋转运动：当机器人匀速旋转时，实际走过的是一段圆弧
    // 预测下一个周期的航向角
    double predicted_yaw = NavigationUtils::normalizeRadian(yaw - angular_velocity / (plan_freq * 2));
    
    // ========== 将全局速度转换到机器人坐标系 ==========
    cmd_vel.linear.x = vx_global * cos(predicted_yaw) + vy_global * sin(predicted_yaw);
    cmd_vel.linear.y = -vx_global * sin(predicted_yaw) + vy_global * cos(predicted_yaw);
    
    // ========== 安全检查：处理NaN值 ==========
    if (std::isnan(cmd_vel.linear.x))
    {
        cmd_vel.linear.x = 0;
    }
    if (std::isnan(cmd_vel.linear.y))
    {
        cmd_vel.linear.y = 0;
    }
}

/**
 * @brief Controller析构函数
 */
Controller::~Controller()
{
}

/**
 * @brief 主函数
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return 程序退出码
 * 
 * 初始化ROS节点，创建控制器实例并进入事件循环
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "pid_position_follow");
    
    // 使用异步spinner提高响应性（4个线程）
    ros::AsyncSpinner spinner(4);
    spinner.start();
    
    // 创建控制器实例（构造函数中会完成所有初始化）
    Controller controller;
    
    // 等待ROS关闭信号
    ros::waitForShutdown();
    
    return 0;
}
