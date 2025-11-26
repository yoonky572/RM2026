#include "tracking/tracking.hpp"

#include <pcl/common/centroid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>

Tracker::Tracker()
    : nh_("~")
    , obstacle_cloud_(new pcl::PointCloud<pcl::PointXYZ>()) {
    nh_.param("freq", freq_, 10);
    nh_.param<std::string>("cloud_name", cloud_topic_, "/obstacle_cloud");
    nh_.param("ClusterTolerance", cluster_tolerance_, 0.5);
    nh_.param("min_cluster_size", min_cluster_size_, 5);
    nh_.param("max_cluster_size", max_cluster_size_, 1000);
    nh_.param<std::string>("output_frame", output_frame_, "map");

    obstacle_sub_ = nh_.subscribe(cloud_topic_, 10, &Tracker::cloudCallback, this);
    centroids_pub_ = nh_.advertise<geometry_msgs::PoseArray>("centroids", 10);
    run_timer_ = nh_.createTimer(ros::Duration(1.0 / static_cast<double>(freq_)),
                                 &Tracker::extractDynamicObstacles, this);
}

void Tracker::extractDynamicObstacles(const ros::TimerEvent&) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr local_cloud(new pcl::PointCloud<pcl::PointXYZ>());

    {
        std::lock_guard<std::mutex> lock(obstacle_mutex_);
        *local_cloud = *obstacle_cloud_;
    }

    if (local_cloud->empty()) {
        return;
    }

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(local_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance_);
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(local_cloud);
    ec.extract(cluster_indices);

    obstacle_poses_.header.frame_id = output_frame_;
    obstacle_poses_.header.stamp = timestamp_;
    obstacle_poses_.poses.clear();

    for (const auto& cluster : cluster_indices) {
        Eigen::Vector4d centroid(0.0, 0.0, 0.0, 0.0);
        pcl::PointCloud<pcl::PointXYZ> cluster_cloud;
        cluster_cloud.reserve(cluster.indices.size());

        for (const auto& idx : cluster.indices) {
            cluster_cloud.push_back((*local_cloud)[idx]);
        }

        if (cluster_cloud.empty()) {
            continue;
        }

        pcl::compute3DCentroid(cluster_cloud, centroid);

        geometry_msgs::Pose pose;
        pose.position.x = centroid[0];
        pose.position.y = centroid[1];
        pose.position.z = centroid[2];
        pose.orientation.w = 1.0;

        obstacle_poses_.poses.push_back(pose);
    }

    if (!obstacle_poses_.poses.empty()) {
        centroids_pub_.publish(obstacle_poses_);
    }
}

void Tracker::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    pcl::PointCloud<pcl::PointXYZ> temp_cloud;
    pcl::fromROSMsg(*msg, temp_cloud);

    {
        std::lock_guard<std::mutex> lock(obstacle_mutex_);
        *obstacle_cloud_ = temp_cloud;
        timestamp_ = msg->header.stamp;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"tracking");
    Tracker tracker;
    ros::spin();
    return 0;
}