#include <pluginlib/class_list_macros.h>
#include "recovery/find_freespace.hpp"
#include "util/costmap_utils.hpp"
#include "util/freespace_finder.hpp"
#include <visualization_msgs/Marker.h>
#include <tf2/utils.h>

PLUGINLIB_EXPORT_CLASS(recovery::FindFreespace, nav_core::RecoveryBehavior)

namespace recovery {

FindFreespace::FindFreespace() : nh_("~") {}

FindFreespace::~FindFreespace() {}

void FindFreespace::initialize(std::string name, 
                                tf2_ros::Buffer* tf, 
                                costmap_2d::Costmap2DROS* global_costmap,
                                costmap_2d::Costmap2DROS* local_costmap)
{
    name_ = name;
    global_costmap_ = global_costmap;
    local_costmap_ = local_costmap;

    // 加载参数
    nh_.param<double>("max_search_radius", max_search_radius_, 0.3);
    nh_.param<double>("speed", escape_speed_, 0.5);
    command_duration_ = 1.0;  // 默认发布命令 1 秒

    // 初始化发布器
    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/escape_vel", 10);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("escape_goal", 10);
}

void FindFreespace::runBehavior()
{
    // 获取代价地图（使用锁保护）
    costmap_2d::Costmap2D* costmap;
    {
        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(
            *(global_costmap_->getCostmap()->getMutex()));
        costmap = global_costmap_->getCostmap();
    }

    if (!costmap) {
        ROS_ERROR("[FindFreespace] Failed to get costmap");
        return;
    }

    double resolution = costmap->getResolution();
    int grid_radius = static_cast<int>(max_search_radius_ / resolution);

    // 获取机器人当前位置
    geometry_msgs::PoseStamped robot_pose;
    if (!global_costmap_->getRobotPose(robot_pose)) {
        ROS_ERROR("[FindFreespace] Failed to get robot pose");
        return;
    }

    double world_x = robot_pose.pose.position.x;
    double world_y = robot_pose.pose.position.y;

    // 转换为栅格坐标
    int grid_x, grid_y;
    if (!util::worldToGrid(world_x, world_y, costmap, grid_x, grid_y)) {
        ROS_ERROR("[FindFreespace] Failed to convert world to grid coordinates or costmap is empty");
        return;
    }

    ROS_DEBUG("[FindFreespace] Robot at grid (%d, %d), world (%.3f, %.3f), cost: %d",
              grid_x, grid_y, world_x, world_y, costmap->getCost(grid_x, grid_y));

    // 检查机器人周围是否有足够的自由空间
    if (isSurroundedByFreeSpace(costmap, grid_x, grid_y)) {
        ROS_INFO("[FindFreespace] Robot is not stuck, no escape needed");
        return;
    }

    // 搜索自由空间和障碍物
    std::vector<geometry_msgs::Point> freespace;
    std::vector<geometry_msgs::Point> obstacles;
    util::findFreespaceAndObstacles(costmap, grid_x, grid_y, grid_radius, 
                                     freespace, obstacles);

    if (freespace.empty()) {
        ROS_ERROR("[FindFreespace] Failed to find free space within search radius");
        return;
    }

    // 计算障碍物中心并找到最远的自由空间点
    geometry_msgs::Point escape_goal_grid;
    double obstacle_center_x = 0.0, obstacle_center_y = 0.0;

    if (util::calculateObstacleCenter(obstacles, obstacle_center_x, obstacle_center_y)) {
        // 有障碍物，选择距离障碍物中心最远的自由空间点
        escape_goal_grid = util::findFarthestFreespace(freespace, 
                                                        obstacle_center_x, 
                                                        obstacle_center_y);
    } else {
        // 没有障碍物，使用第一个自由空间点
        escape_goal_grid = freespace.front();
    }

    // 将栅格坐标转换为世界坐标
    geometry_msgs::Point escape_goal_world;
    if (!util::gridToWorld(static_cast<int>(escape_goal_grid.x), 
                           static_cast<int>(escape_goal_grid.y),
                           costmap,
                           escape_goal_world.x,
                           escape_goal_world.y)) {
        ROS_ERROR("[FindFreespace] Failed to convert grid to world coordinates");
        return;
    }
    escape_goal_world.z = 0.0;

    ROS_INFO("[FindFreespace] Escape goal at (%.3f, %.3f)", 
             escape_goal_world.x, escape_goal_world.y);

    // 发布速度命令
    publishVelocityCommand(robot_pose, escape_goal_world, command_duration_);

    // 发布可视化标记
    publishMarker(escape_goal_world, robot_pose.header.frame_id);
}

bool FindFreespace::isSurroundedByFreeSpace(const costmap_2d::Costmap2D* costmap, 
                                             int grid_x, int grid_y) const
{
    int size_x = costmap->getSizeInCellsX();
    int size_y = costmap->getSizeInCellsY();

    // 检查四个方向是否都是自由空间
    bool right_free = (grid_x + 1 < size_x) && 
                      (costmap->getCost(grid_x + 1, grid_y) == costmap_2d::FREE_SPACE);
    bool left_free = (grid_x - 1 >= 0) && 
                     (costmap->getCost(grid_x - 1, grid_y) == costmap_2d::FREE_SPACE);
    bool up_free = (grid_y + 1 < size_y) && 
                   (costmap->getCost(grid_x, grid_y + 1) == costmap_2d::FREE_SPACE);
    bool down_free = (grid_y - 1 >= 0) && 
                     (costmap->getCost(grid_x, grid_y - 1) == costmap_2d::FREE_SPACE);

    return right_free && left_free && up_free && down_free;
}

void FindFreespace::publishVelocityCommand(const geometry_msgs::PoseStamped& robot_pose,
                                            const geometry_msgs::Point& target_position,
                                            double duration)
{
    // 计算目标方向（全局坐标系）
    double target_yaw = std::atan2(target_position.y - robot_pose.pose.position.y,
                                    target_position.x - robot_pose.pose.position.x);

    // 获取机器人当前朝向
    double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);

    // 计算全局坐标系下的速度分量
    double vx_global = escape_speed_ * std::cos(target_yaw);
    double vy_global = escape_speed_ * std::sin(target_yaw);

    // 转换到机器人局部坐标系
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = vx_global * std::cos(robot_yaw) + vy_global * std::sin(robot_yaw);
    cmd_vel.linear.y = -vx_global * std::sin(robot_yaw) + vy_global * std::cos(robot_yaw);
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0;

    // 持续发布速度命令
    ros::Time start_time = ros::Time::now();
    ros::Rate rate(100);  // 100 Hz

    while ((ros::Time::now() - start_time).toSec() < duration) {
        cmd_pub_.publish(cmd_vel);
        rate.sleep();
    }

    ROS_DEBUG("[FindFreespace] Published velocity command: linear.x=%.3f, linear.y=%.3f",
              cmd_vel.linear.x, cmd_vel.linear.y);
}

void FindFreespace::publishMarker(const geometry_msgs::Point& position, 
                                   const std::string& frame_id)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "escape_goal";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position = position;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;

    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(50.0);

    marker_pub_.publish(marker);
}

} // namespace recovery
