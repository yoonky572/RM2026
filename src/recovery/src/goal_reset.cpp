#include <pluginlib/class_list_macros.h>
#include "recovery/goal_reset.hpp"
#include "util/costmap_utils.hpp"
#include "util/freespace_finder.hpp"
#include <visualization_msgs/Marker.h>

PLUGINLIB_EXPORT_CLASS(recovery::Goal_Reset, nav_core::RecoveryBehavior)

namespace recovery {

Goal_Reset::Goal_Reset() : nh_("~") {}

Goal_Reset::~Goal_Reset() {}

void Goal_Reset::initialize(std::string name, 
                            tf2_ros::Buffer* tf, 
                            costmap_2d::Costmap2DROS* global_costmap,
                            costmap_2d::Costmap2DROS* local_costmap)
{
    name_ = name;
    global_costmap_ = global_costmap;
    local_costmap_ = local_costmap;

    // 加载参数
    nh_.param<double>("max_search_radius", max_search_radius_, 0.3);

    // 初始化发布器和订阅器
    goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("new_goal", 10);
    goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
        "/move_base_simple/goal", 1, &Goal_Reset::goalCallback, this);
}

void Goal_Reset::runBehavior()
{
    // 获取代价地图
    auto costmap = global_costmap_->getCostmap();
    if (!costmap) {
        ROS_ERROR("[Goal_Reset] Failed to get costmap");
        return;
    }

    double resolution = costmap->getResolution();
    int grid_radius = static_cast<int>(max_search_radius_ / resolution);

    // 获取目标点的世界坐标
    double world_x = initial_goal_.pose.position.x;
    double world_y = initial_goal_.pose.position.y;

    // 转换为栅格坐标
    int grid_x, grid_y;
    if (!util::worldToGrid(world_x, world_y, costmap, grid_x, grid_y)) {
        ROS_ERROR("[Goal_Reset] Failed to convert world to grid coordinates or costmap is empty");
        return;
    }

    ROS_DEBUG("[Goal_Reset] Target at grid (%d, %d), world (%.3f, %.3f)", 
              grid_x, grid_y, world_x, world_y);

    // 检查目标点是否在自由空间中
    if (costmap->getCost(grid_x, grid_y) == costmap_2d::FREE_SPACE) {
        ROS_INFO("[Goal_Reset] Goal is in free space, no reset needed");
        return;
    }

    // 搜索自由空间和障碍物
    std::vector<geometry_msgs::Point> freespace;
    std::vector<geometry_msgs::Point> obstacles;
    util::findFreespaceAndObstacles(costmap, grid_x, grid_y, grid_radius, 
                                     freespace, obstacles);

    if (freespace.empty()) {
        ROS_ERROR("[Goal_Reset] Failed to find free space within search radius");
        return;
    }

    // 计算障碍物中心并找到最远的自由空间点
    geometry_msgs::Point new_goal_grid;
    double obstacle_center_x = 0.0, obstacle_center_y = 0.0;

    if (util::calculateObstacleCenter(obstacles, obstacle_center_x, obstacle_center_y)) {
        // 有障碍物，选择距离障碍物中心最远的自由空间点
        new_goal_grid = util::findFarthestFreespace(freespace, 
                                                     obstacle_center_x, 
                                                     obstacle_center_y);
    } else {
        // 没有障碍物，使用第一个自由空间点
        new_goal_grid = freespace.front();
    }

    // 将栅格坐标转换为世界坐标
    geometry_msgs::PoseStamped new_goal_world;
    new_goal_world.header.frame_id = initial_goal_.header.frame_id;
    new_goal_world.header.stamp = ros::Time::now();
    new_goal_world.pose.orientation = initial_goal_.pose.orientation;

    if (!util::gridToWorld(static_cast<int>(new_goal_grid.x), 
                           static_cast<int>(new_goal_grid.y),
                           costmap,
                           new_goal_world.pose.position.x,
                           new_goal_world.pose.position.y)) {
        ROS_ERROR("[Goal_Reset] Failed to convert grid to world coordinates");
        return;
    }

    // 发布新目标点
    goal_pub_.publish(new_goal_world);
    ROS_INFO("[Goal_Reset] Reset goal to (%.3f, %.3f)", 
             new_goal_world.pose.position.x, 
             new_goal_world.pose.position.y);

    // 发布可视化标记
    publishMarker(new_goal_world.pose.position, new_goal_world.header.frame_id);
}

void Goal_Reset::goalCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    initial_goal_ = *msg;
}

void Goal_Reset::publishMarker(const geometry_msgs::Point& position, 
                                const std::string& frame_id)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "new_goal";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position = position;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.03;
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;

    marker.color.r = 1.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(5.0);

    marker_pub_.publish(marker);
}

} // namespace recovery
