/**
 * @file costmap_clean.cpp
 * @brief 周期性清除代价地图的服务客户端
 * 
 * 该程序定期调用move_base的清除代价地图服务，用于测试或定期维护。
 */

#include <ros/ros.h>
#include <std_srvs/Empty.h>

/**
 * @brief 主函数
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return 程序退出码
 * 
 * 初始化ROS节点后，每6秒调用一次代价地图清除服务
 */
int main(int argc, char** argv)
{
    ROS_INFO("Starting costmap clearing service client...");
    
    ros::init(argc, argv, "clear_costmap_service");
    ros::NodeHandle nh;
    
    // 等待服务可用
    const std::string SERVICE_NAME = "/move_base/clear_costmaps";
    ros::service::waitForService(SERVICE_NAME);
    
    // 创建服务客户端
    ros::ServiceClient clear_client = nh.serviceClient<std_srvs::Empty>(SERVICE_NAME);
    std_srvs::Empty srv;
    
    // 周期性清除代价地图
    const double CLEAR_INTERVAL = 6.0;  // 清除间隔（秒）
    
    while (ros::ok())
    {
        if (clear_client.call(srv))
        {
            ROS_INFO("Costmap cleared successfully");
        }
        else
        {
            ROS_WARN("Failed to clear costmap");
        }
        
        ros::Duration(CLEAR_INTERVAL).sleep();
    }
    
    return 0;
}