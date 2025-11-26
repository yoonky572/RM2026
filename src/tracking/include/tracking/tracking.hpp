#pragma once

#include <mutex>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/**
 * @brief Tracker 类：对输入点云做聚类，输出每个障碍物的质心
 */
class Tracker {
public:
    Tracker();
    ~Tracker() = default;

private:
    void extractDynamicObstacles(const ros::TimerEvent& event);
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

    // Parameters
    int freq_{10};
    double cluster_tolerance_{0.5};
    int min_cluster_size_{5};
    int max_cluster_size_{1000};
    std::string cloud_topic_;
    std::string output_frame_;

    // ROS interfaces
    ros::NodeHandle nh_;
    ros::Subscriber obstacle_sub_;
    ros::Publisher centroids_pub_;
    ros::Timer run_timer_;

    // Data
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud_;
    geometry_msgs::PoseArray obstacle_poses_;
    ros::Time timestamp_;
    std::mutex obstacle_mutex_;
};
