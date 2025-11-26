#pragma once

#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <cmath>

namespace recovery {
namespace util {

/**
 * @brief 在指定半径内搜索自由空间和障碍物
 * @param costmap 代价地图指针
 * @param center_x 搜索中心栅格坐标 x
 * @param center_y 搜索中心栅格坐标 y
 * @param search_radius 搜索半径（栅格单位）
 * @param freespace 输出的自由空间点集合
 * @param obstacles 输出的障碍物点集合
 */
inline void findFreespaceAndObstacles(
    const costmap_2d::Costmap2D* costmap,
    int center_x, int center_y,
    int search_radius,
    std::vector<geometry_msgs::Point>& freespace,
    std::vector<geometry_msgs::Point>& obstacles)
{
    freespace.clear();
    obstacles.clear();

    int size_x = costmap->getSizeInCellsX();
    int size_y = costmap->getSizeInCellsY();

    // 从中心向外逐层搜索（与原始实现保持一致，允许重复检查内层点）
    for (int radius = 0; radius <= search_radius; ++radius) {
        for (int x = center_x - radius; x <= center_x + radius; ++x) {
            for (int y = center_y - radius; y <= center_y + radius; ++y) {
                // 跳过边界外的点
                if (x < 0 || x >= size_x || y < 0 || y >= size_y) {
                    continue;
                }

                unsigned char cost = costmap->getCost(x, y);
                
                if (cost == costmap_2d::FREE_SPACE) {
                    geometry_msgs::Point point;
                    point.x = static_cast<double>(x);
                    point.y = static_cast<double>(y);
                    point.z = 0.0;
                    freespace.push_back(point);
                } else if (cost == costmap_2d::LETHAL_OBSTACLE) {
                    geometry_msgs::Point point;
                    point.x = static_cast<double>(x);
                    point.y = static_cast<double>(y);
                    point.z = 0.0;
                    obstacles.push_back(point);
                }
            }
        }
    }
}

/**
 * @brief 计算障碍物中心点
 * @param obstacles 障碍物点集合
 * @param center_x 输出的中心点 x 坐标
 * @param center_y 输出的中心点 y 坐标
 * @return 是否成功计算（障碍物集合非空）
 */
inline bool calculateObstacleCenter(
    const std::vector<geometry_msgs::Point>& obstacles,
    double& center_x, double& center_y)
{
    if (obstacles.empty()) {
        return false;
    }

    double x_sum = 0.0;
    double y_sum = 0.0;

    for (const auto& point : obstacles) {
        x_sum += point.x;
        y_sum += point.y;
    }

    center_x = x_sum / obstacles.size();
    center_y = y_sum / obstacles.size();

    return true;
}

/**
 * @brief 找到距离障碍物中心最远的自由空间点
 * @param freespace 自由空间点集合
 * @param obstacle_center_x 障碍物中心 x 坐标
 * @param obstacle_center_y 障碍物中心 y 坐标
 * @return 最远的自由空间点，如果集合为空则返回第一个点
 */
inline geometry_msgs::Point findFarthestFreespace(
    const std::vector<geometry_msgs::Point>& freespace,
    double obstacle_center_x, double obstacle_center_y)
{
    if (freespace.empty()) {
        geometry_msgs::Point empty_point;
        empty_point.x = 0.0;
        empty_point.y = 0.0;
        empty_point.z = 0.0;
        return empty_point;
    }

    // 如果没有障碍物中心，返回第一个自由空间点
    if (freespace.size() == 1) {
        return freespace.front();
    }

    double max_dist = 0.0;
    geometry_msgs::Point farthest_point = freespace.front();

    for (const auto& point : freespace) {
        double dist = std::hypot(point.x - obstacle_center_x, 
                                 point.y - obstacle_center_y);
        if (dist > max_dist) {
            max_dist = dist;
            farthest_point = point;
        }
    }

    return farthest_point;
}

} // namespace util
} // namespace recovery

