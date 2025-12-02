#include "navigation/controller.hpp"

/**
 * @brief Controller构造函数
 * 初始化ROS参数、发布器、订阅器和定时器
 */
Controller::Controller()
{
    ros::NodeHandle nh("~");
    
    // ========== 加载ROS参数 ==========
    // 最大线速度（m/s）：限制机器人运动的最大速度，防止过快导致不稳定
    nh.param<double>("max_speed", max_speed, 1.0);
    
    // 速度比例系数：控制速度响应灵敏度，值越大速度响应越快
    // 速度计算公式：v = distance * p_value
    nh.param<double>("p_value", p_value, 0.5);
    
    // 无规划时的角速度（rad/s）：当没有路径规划时，机器人原地旋转的角速度
    // 设为0表示停止，非0表示持续旋转（可用于搜索）
    nh.param<double>("set_yaw_speed", set_yaw_speed, 0);
    
    // 规划循环频率（Hz）：Plan函数执行的频率，值越大控制响应越快，但计算负担也越大
    nh.param<int>("plan_frequency", plan_freq, 30);
    
    // 到达目标的距离阈值（m）：当机器人到目标点的距离小于此值时，判定为到达目标
    nh.param<double>("goal_dist_tolerance", goal_dist_tolerance, 0.2);
    
    // 前视距离阈值（m）：在寻找最近路径点时，要求路径点距离机器人至少此距离
    // 用于提供一定前视，使路径跟踪更平滑
    nh.param<double>("prune_ahead_distance", prune_ahead_dist, 0.5);
    
    // 全局坐标系名称：全局路径所在的坐标系，通常为"map"
    nh.param<std::string>("global_frame", global_frame, "map");
    
    // 初始化发布器和订阅器
    local_path_pub = nh.advertise<nav_msgs::Path>("local_path", 5);
    global_path_sub = nh.subscribe("/move_base/GlobalPlanner/plan", 5,
                                    &Controller::GlobalPathCallback, this);
    act_command_sub = nh.subscribe("/diverge", 5,
                                    &Controller::LocalizationStatusCallback, this);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    
    // 初始化TF监听器和定时器
    tf_listener = std::make_shared<tf::TransformListener>();
    plan_timer = nh.createTimer(ros::Duration(1.0 / plan_freq), &Controller::Plan, this);
    
    // 初始化状态变量
    prune_index = 0;
    
    ROS_INFO("Controller initialized with max_speed = %.2f", max_speed);
}

/**
 * @brief 主规划控制循环
 * @param event 定时器事件
 * 
 * plan=1时开启路径追踪，plan=0时原地旋转
 */
void Controller::Plan(const ros::TimerEvent& event)
{
    if (plan)
    {
        // 获取机器人在global_frame下的当前位姿
        geometry_msgs::PoseStamped robot_pose;
        GetTargetRobotPose(tf_listener, global_path.header.frame_id, robot_pose);
        
        // 检查是否到达目标点
        double distance_to_goal = GetEuclideanDistance(robot_pose, global_path.poses.back());
        if (distance_to_goal <= goal_dist_tolerance || prune_index == global_path.poses.size() - 1)
        {
            plan = false;
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = 0;
            cmd_vel.linear.y = 0;
            cmd_vel.angular.z = 0;
            cmd_vel_pub.publish(cmd_vel);
            ROS_INFO("Planning Success!");
            prune_index = 0;
            return;
        }
        
        // 在全局路径上找到距离机器人最近的路径点
        FindNearstPose(robot_pose, global_path, prune_index, prune_ahead_dist);
        
        // 从全局路径中截取一段局部路径（从机器人当前位置往后20个点）
        nav_msgs::Path prune_path, local_path;
        prune_path.header.frame_id = global_frame;
        prune_path.poses.push_back(robot_pose);

        int i = prune_index;
        // 前视索引：从全局路径中提取的路径点数量（从当前点往后提取多少个点）
        // 这些点将用于生成局部平滑轨迹
        const int FORESEE_INDEX = 20;
        while (i < static_cast<int>(global_path.poses.size()) && i - prune_index < FORESEE_INDEX)
        {
            prune_path.poses.push_back(global_path.poses[i]);
            i++;
        }

        // 调用GenTraj函数根据prune_path生成平滑的局部轨迹local_path
        GenTraj(prune_path, local_path);
        local_path_pub.publish(local_path);

        // 根据局部路径与位姿，计算并发布速度控制指令
        geometry_msgs::Twist cmd_vel;
        FollowTraj(robot_pose, local_path, cmd_vel);
        cmd_vel_pub.publish(cmd_vel);
    }   
    else
    {
        // 无规划时，原地旋转（如果设置了set_yaw_speed）
        geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = 0;
            cmd_vel.linear.y = 0;
            cmd_vel.angular.z = set_yaw_speed;
        cmd_vel.linear.z = 0;  // bool success or not
            cmd_vel_pub.publish(cmd_vel);
    }
}

/**
 * @brief 全局路径回调函数
 * @param msg 接收到的全局路径消息
 * 
 * 当接收到新的全局路径时，存储全局路径并启用路径跟踪
 */
void Controller::GlobalPathCallback(const nav_msgs::PathConstPtr& msg)
{
    if (!msg->poses.empty())
    {
        global_path = *msg;  // 存储全局路径
        
        // 只有未发生定位发散时才启用路径跟踪
        if (diverge == false)
        {
            plan = true;  // 启用路径跟踪
  }
}
}

/**
 * @brief 定位状态回调函数
 * @param msg 接收到的定位状态消息（/diverge话题）
 * 
 * 当定位发散时（msg->data == 1），停止路径跟踪
 */
void Controller::LocalizationStatusCallback(const std_msgs::BoolConstPtr& msg)
{
    if (msg->data == 1)  // 如果定位发散
    {
        diverge = true;   // 设置标志位
        plan = false;     // 停止路径跟踪
    }
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
    // 距离阈值（米）：搜索最近路径点的距离上限，超过此距离认为路径点无效
    // 通常机器人不会偏离路径超过10米，因此设置此阈值避免搜索过远
    const double DIST_THRESHOLD = 10.0;
    const double SQ_DIST_THRESHOLD = DIST_THRESHOLD * DIST_THRESHOLD;
    
            double sq_dist;
    // 初始化距离：如果有前一个路径点，使用前一个点的距离作为初始值
    // 否则使用大值（1e10），表示从路径起点开始搜索
    if (prune_index != 0)
    {
        sq_dist = GetEuclideanDistance(robot_pose, path.poses[prune_index - 1]);
    }
    else
    {
        sq_dist = 1e10;  // 初始大值，表示从路径起点开始搜索
            }

    // 向前搜索最近的路径点
            double new_sq_dist = 0;
    while (prune_index < static_cast<int>(path.poses.size()))
    {
        new_sq_dist = GetEuclideanDistance(robot_pose, path.poses[prune_index]);
        
        // 如果距离开始增加且之前的距离在阈值内，说明已经找到最近点
        if (new_sq_dist > sq_dist && sq_dist < SQ_DIST_THRESHOLD)
        {
            // 判断路径点是否在同方向且距离足够远（大于prune_ahead_dist）
            // 计算当前点和前一个点相对于机器人的方向向量
            double dx_current = path.poses[prune_index].pose.position.x - robot_pose.pose.position.x;
            double dy_current = path.poses[prune_index].pose.position.y - robot_pose.pose.position.y;
            double dx_previous = path.poses[prune_index - 1].pose.position.x - robot_pose.pose.position.x;
            double dy_previous = path.poses[prune_index - 1].pose.position.y - robot_pose.pose.position.y;
            
            // 判断两个路径点是否在同一方向（点积>0表示同向）
            bool same_direction = (dx_current * dx_previous + dy_current * dy_previous > 0);
            // 判断距离是否足够远（大于前视距离阈值）
            bool sufficient_distance = sq_dist > prune_ahead_dist * prune_ahead_dist;
            
            // 如果满足条件（同方向且距离足够），回退一个索引以获得更安全的前视点
            // 这样可以提供一定前瞻，使路径跟踪更平滑
            if (same_direction && sufficient_distance)
            {
                        prune_index--;
            }
            else
            {
                        sq_dist = new_sq_dist;
                    }

                    break;
                }
        
                sq_dist = new_sq_dist;
                ++prune_index;
            }
    
    // 确保索引不超出范围
    prune_index = std::min(prune_index, static_cast<int>(path.poses.size() - 1));
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
    // 获取机器人当前航向角（弧度）：从四元数中提取yaw角
        yaw = tf::getYaw(robot_pose.pose.orientation);

    // 计算机器人到局部路径中第一个目标点的方向角
    // 注意：traj.poses[0]是机器人当前位置，traj.poses[1]是下一个目标点
    double dx = traj.poses[1].pose.position.x - robot_pose.pose.position.x;
    double dy = traj.poses[1].pose.position.y - robot_pose.pose.position.y;
    double diff_yaw = atan2(dy, dx);  // 计算目标方向角（全局坐标系）
    
    // 归一化角度到[-π, π]范围，确保角度在合理范围内
    diff_yaw = normalizeRadian(diff_yaw);
    
    // 计算到下一个路径点的欧氏距离（米）
    double diff_distance = GetEuclideanDistance(robot_pose, traj.poses[1]);

    // 计算全局坐标系下的速度
    // 速度大小与距离成正比：v = distance * p_value
    // 速度方向指向目标点：(cos(diff_yaw), sin(diff_yaw))
    double vx_global = cos(diff_yaw) * diff_distance * p_value;
    double vy_global = sin(diff_yaw) * diff_distance * p_value;
    
    // 速度限幅：限制最大速度不超过max_speed
    // 注意：保持原代码逻辑（使用sqrt(vx)+sqrt(vy)判断，原代码注释指出可能有问题）
    // 如果速度过大，按最大速度max_speed重新计算速度方向
    if (sqrt(vx_global) + sqrt(vy_global) > sqrt(max_speed))
        {
        vx_global = max_speed * cos(diff_yaw);
        vy_global = max_speed * sin(diff_yaw);
        }
    
    // 将全局坐标系下的速度转换到机器人坐标系（base_link）
    // 旋转矩阵：[cos(yaw)  sin(yaw) ] [vx_global]
    //          [-sin(yaw) cos(yaw) ] [vy_global]
    cmd_vel.linear.x = vx_global * cos(yaw) + vy_global * sin(yaw);  // 前进速度
    cmd_vel.linear.y = -vx_global * sin(yaw) + vy_global * cos(yaw); // 横向速度
    cmd_vel.angular.z = set_yaw_speed;  // 角速度（通常为0，由底盘控制航向）
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
  Controller controller;
  ros::spin();
    return 0;
}
