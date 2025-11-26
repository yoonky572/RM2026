#pragma once

#include <costmap_2d/costmap_2d.h>

namespace recovery {
namespace util {

/**
 * @brief 将世界坐标转换为栅格坐标
 * @param wx 世界坐标 x
 * @param wy 世界坐标 y
 * @param costmap 代价地图指针
 * @param gx 输出的栅格坐标 x
 * @param gy 输出的栅格坐标 y
 * @return 转换是否成功（坐标在有效范围内）
 */
inline bool worldToGrid(double wx, double wy, 
                        const costmap_2d::Costmap2D* costmap, 
                        int& gx, int& gy) 
{
    if (costmap->getSizeInCellsX() == 0 || costmap->getSizeInCellsY() == 0) {
        return false;
    }

    gx = static_cast<int>((wx - costmap->getOriginX()) / costmap->getResolution());
    gy = static_cast<int>((wy - costmap->getOriginY()) / costmap->getResolution());

    return (gx >= 0 && gx < costmap->getSizeInCellsX() && 
            gy >= 0 && gy < costmap->getSizeInCellsY());
}

/**
 * @brief 将栅格坐标转换为世界坐标
 * @param gx 栅格坐标 x
 * @param gy 栅格坐标 y
 * @param costmap 代价地图指针
 * @param wx 输出的世界坐标 x
 * @param wy 输出的世界坐标 y
 * @return 转换是否成功（代价地图有效）
 */
inline bool gridToWorld(int gx, int gy, 
                        const costmap_2d::Costmap2D* costmap,
                        double& wx, double& wy)
{
    if (costmap->getSizeInCellsX() == 0 || costmap->getSizeInCellsY() == 0) {
        return false;
    }

    wx = (gx + 0.5) * costmap->getResolution() + costmap->getOriginX();
    wy = (gy + 0.5) * costmap->getResolution() + costmap->getOriginY();

    return true;   
}

} // namespace util
} // namespace recovery


