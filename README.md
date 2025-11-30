代码启动脚本
```bash
#!/bin/bash

sleep 1
source /opt/ros/noetic/setup.bash
source /home/tjurm/navi_ws/Sentry25NAVI/devel/setup.bash
source /home/tjurm/linefit_ws/devel/setup.bash

# 1. 硬件驱动层：启动 Livox 激光雷达
roslaunch /home/tjurm/navi_ws/Sentry25NAVI/src/livox_ros_driver2-master/launch_ROS1/msg_MID360.launch > livox.log 2>&1 &

sleep 2

# 2. 核心控制层：
# 这个节点会级联启动 Point-LIO(定位), MoveBase(规划), Controller(控制) 等
roslaunch /home/tjurm/navi_ws/Sentry25NAVI/src/sentry_simulation/launch/start.launch > simulation.log 2>&1 &

sleep 2

# 3. 感知层：启动地面分割算法
roslaunch /home/tjurm/linefit_ws/src/linefit_ground_segmentation/linefit_ground_segmentation_ros/launch/segmentation.launch > segmentation.log 2>&1 &

sleep 1

# 4. 驱动与通讯层：启动串口通信（连接下位机 STM32）
roslaunch /home/tjurm/navi_ws/Sentry25NAVI/src/tracking/launch/serial.launch > serial.log 2>&1 &

wait
exit 0
```

---

### 2. 详细工作流分析 

#### 第一阶段：感知与硬件输入 

*   **Livox MID360 **:
    *   **操作**: `msg_MID360.launch` 启动雷达驱动。
    *   **数据**: 产生原始激光点云和内置 IMU 数据。
*   **LineFit Ground Segmentation (Preprocessing)**:
    *   **操作**: `segmentation.launch` 订阅原始点云。
    *   **逻辑**: 比赛场地虽平坦但有坡度，算法通过几何特征将点云分为 **地面点** 和 **障碍物点**。
    *   **输出**: 
        *   `obstacle_cloud`: 仅包含障碍物的点云，自定义的segmentation功能包接收的就是这个点云，用这个点云去和先验静态地图作对比，区分静态障碍物（墙）和动态障碍物（人）。从而输出动态障碍物点云。
        *   `ground_cloud`: 地面点云

#### 第二阶段：状态估计 

*   **Point-LIO (Localization)**:
    *   **操作**: 由 `launch_controller.py` 内部调用 `mapping_mid360.launch` 启动。
    *   **逻辑**: 融合高频 IMU 和激光点云，进行实时 SLAM。
    *   **输出**: 
        姿态与点云
    *   `launch_controller.py` 会监控定位状态，如果定位丢失可能会触发重启。

#### 第三阶段：规划与决策 

*   **Move Base**:
    *   **操作**: 由 `simu_navi.launch` 启动。
    *   **逻辑**: 维护全局代价地图（Global Costmap）和局部代价地图（Local Costmap）。接收 `obstacle_cloud` 更新障碍物信息。
    *   **输出**: 根据起点和终点，使用  A* 算法生成一条无碰撞的 **全局路径 (Global Path)**。
*   **Goal Distribute / Serial Decision**:
    *   **操作**: 目标点由上层逻辑决定。
    *   **逻辑**: `tracking/serial.launch` 或 `goal_distribute` 负责将逻辑目标（如“去高地”）转换为具体的地图坐标 (x, y) 发送给 MoveBase。有个yaml文件里设定了目标点编号和对应坐标

#### 第四阶段：运动控制 

*   **Controller **:
    *   **操作**: 运行 `navigation/src/controller.cpp`。
    *   **输入**: 
        1.  MoveBase 生成的全局路径。
        2.  Point-LIO 提供的实时位姿。
    *   **逻辑**: 
        *   **路径裁剪与平滑**: 截取机器人前方的局部路径。
        *   **纯追踪/MPC**: 计算应该以多大的 x, y 速度和自旋角速度 (w) 来最快贴合路径。
        *   **动态避障**: 如果发现路径上有新出现的障碍物（Costmap更新），进行局部微调。
    *   **输出**: `/cmd_vel` (几何速度指令)。

#### 第五阶段：执行与反馈 

*   **Serial **:
    *   **操作**: `serial.launch` (即 `tracking` 包中的 `serial.cpp`)。
    *   **输入**: 订阅 `/cmd_vel`。
    *   **逻辑**: 
        *   将 ROS 的浮点数速度指令转换为底层单片机（STM32）能识别的数据帧协议（通常包含包头、校验位等）。
        *   **数据**: `vx`, `vy`, `omega` (自旋速度)。
    *   **双向通讯**: 它同时也读取数据，包含：
        *   裁判系统数据（扣血、比赛阶段）。
        *   自身颜色信息（红/蓝方）

---
