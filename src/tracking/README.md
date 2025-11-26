# Tracking 障碍物聚类包

## 概述

`tracking` 包对输入的障碍物点云进行欧式聚类并输出每个障碍物的几何质心，可用于后续的目标跟踪或策略模块。

## 运行流程

1. 订阅障碍物点云（`sensor_msgs/PointCloud2`）
2. 将点云缓存在内部，定时触发处理
3. 使用 `pcl::EuclideanClusterExtraction` 做聚类
4. 对每个聚类点云计算 3D 质心
5. 将质心集合通过 `geometry_msgs::PoseArray` 发布

## 关键话题

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `cloud_name` 参数指定（默认 `/detector/obstacle`） | `sensor_msgs/PointCloud2` | 订阅 | 障碍物点云输入 |
| `/tracking/centroids` （命名空间取决于节点名） | `geometry_msgs/PoseArray` | 发布 | 每个聚类的质心，坐标系由 `output_frame` 指定 |

## 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `freq` | int | 10 | 处理频率（Hz） |
| `cloud_name` | string | `/detector/obstacle` | 输入点云话题 |
| `ClusterTolerance` | double | 0.5 | 聚类半径（米） |
| `min_cluster_size` | int | 5 | 聚类最小点数 |
| `max_cluster_size` | int | 1000 | 聚类最大点数 |
| `output_frame` | string | `map` | 发布质心使用的坐标系 |

## 启动示例

```xml
<launch>
  <node pkg="tracking" type="tracking" name="tracking" output="screen">
    <param name="freq" value="5" />
    <param name="cloud_name" value="/detector/obstacle" />
    <param name="ClusterTolerance" value="0.3" />
    <param name="min_cluster_size" value="5" />
    <param name="max_cluster_size" value="500" />
    <param name="output_frame" value="map" />
  </node>
</launch>
```

## 重构说明

本次重构完成的主要工作：

1. **代码整理**：删除未使用的成员与空函数，采用 RAII 管理并添加线程安全注释。
2. **参数化**：聚类半径、聚类点数范围、输出坐标系均可配置，与原有逻辑保持一致。
3. **注释完善**：为核心流程补充注释，便于维护。
4. **功能保持**：输入输出话题、聚类算法与原实现完全一致。

## 依赖

- ROS
- PCL（用于点云聚类）

编译方式与常规 catkin 包一致：

```bash
catkin_make
# 或
catkin build tracking
```

