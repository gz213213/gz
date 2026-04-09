# chapt8_ws 导航流程实战解析（基于当前项目）

这份文档不是通用教程，而是按你当前工程配置整理的“能对上代码、能跑起来、能排障”的导航流程说明。

## 1. 先看全链路（从目标点到车轮）

```text
RViz 2D Goal Pose
  -> bt_navigator（行为树调度）
  -> planner_server（SmacPlanner2D 生成全局路径）
  -> smoother_server（SimpleSmoother 平滑路径）
  -> controller_server（RPP 生成速度指令 cmd_vel_nav）
  -> velocity_smoother（cmd_vel_nav -> /cmd_vel）
  -> fishbot_diff_drive_controller（底盘执行）
  -> Gazebo 机器人运动
  -> /odom + odom->base_footprint TF（里程计闭环）

同时：
map_server 发布静态地图
amcl 消费 /scan + /tf + /odom，发布 map->odom TF（定位闭环）
robot_state_publisher 发布 base_footprint->base_link->laser_link 等机体 TF
```

一句话理解：  
导航是否能跑通，本质看两条闭环是否同时成立：
- 定位闭环：`map -> odom -> base_footprint`
- 控制闭环：`goal -> path -> cmd_vel -> odom`

## 2. 你的项目里，谁负责什么

| 模块 | 文件 | 关键作用 |
| --- | --- | --- |
| Gazebo 启动 | `src/fishbot_description/launch/gazebo_sim.launch.py` | 启动仿真、加载机器人、按顺序激活控制器 |
| 机器人模型 | `src/fishbot_description/urdf/fishbot/fishbot.urdf.xacro` | 组装底盘、轮子、激光、IMU、相机、Gazebo 插件 |
| ros2_control 接入 | `src/fishbot_description/urdf/fishbot/fishbot.ros2_control.xacro` | 把 `/cmd_vel` 接到底盘控制器，输出 `/odom` |
| 底盘控制参数 | `src/fishbot_description/config/fishbot_ros2_controller.yaml` | diff drive 轮距/轮径、速度加速度限幅、odom TF |
| Nav2 启动入口 | `src/fishbot_navigation2/launch/navigation2.launch.py` | 传入地图和参数，include `nav2_bringup/bringup_launch.py` |
| Nav2 参数 | `src/fishbot_navigation2/config/nav2_params.yaml` | AMCL / costmap / planner / controller / recovery 等核心配置 |
| 稳定版行为树 | `src/fishbot_navigation2/behavior_trees/navigate_to_pose_stable.xml` | 低频重规划 + 先平滑后跟踪 + 温和恢复策略 |
| 地图 | `src/fishbot_navigation2/maps/room.yaml` | 栅格地图、分辨率、原点 |

## 3. 按时间顺序理解一次完整导航

1. 启动 `gazebo_sim.launch.py`。  
2. `robot_state_publisher` 发布机器人 TF 树（`base_footprint -> base_link -> laser_link ...`）。  
3. 机器人实体被 `spawn_entity.py` 放入 Gazebo。  
4. `controller_manager` 依次激活：  
`fishbot_joint_state_broadcaster` -> `fishbot_diff_drive_controller`。  
5. 底盘控制器开始接收 `/cmd_vel`、发布 `/odom` 与 `odom -> base_footprint` TF。  
6. 启动 `navigation2.launch.py`，进入 Nav2 bringup：  
`map_server + amcl + planner + smoother + controller + bt_navigator + velocity_smoother`。  
7. AMCL 用 `map + /scan + /odom` 估计位姿，发布 `map -> odom` TF。  
8. 在 RViz 下发目标后，BT 执行 `ComputePathToPose -> SmoothPath -> FollowPath`，速度链路输出到 `/cmd_vel`，机器人开始导航。

## 4. 你这套参数为什么更稳（核心思路）

- 规划层：`SmacPlanner2D`，`cost_travel_multiplier: 3.0`，更偏向远离高代价区，走廊更容易走中轴。  
- 平滑层：BT 里显式 `SmoothPath`，减少折线轨迹导致的方向抖动。  
- 控制层：`RPP`（Regulated Pure Pursuit）+ 低速参数（`desired_linear_vel: 0.10`），牺牲峰值速度换稳定性。  
- 局部代价地图：`observation_persistence: 0.0`，减少“幽灵障碍”长期滞留。  
- 恢复行为：先清图/后退/等待，再旋转，避免贴墙时越转越卡。  
- 定位启动：AMCL `set_initial_pose: true`，减少刚启动时 `map->odom` 缺失导致“有路径但不动”。

## 5. 运行与验证（建议按这个顺序）

终端 1（仿真）：
```bash
cd /home/guzhen/chapt8/chapt8_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select fishbot_description fishbot_navigation2
source install/setup.bash
ros2 launch fishbot_description gazebo_sim.launch.py
```

终端 2（导航）：
```bash
cd /home/guzhen/chapt8/chapt8_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch fishbot_navigation2 navigation2.launch.py
```

终端 3（检查链路）：
```bash
source /opt/ros/humble/setup.bash
source /home/guzhen/chapt8/chapt8_ws/install/setup.bash

ros2 node list | rg "amcl|map_server|planner_server|controller_server|bt_navigator|velocity_smoother"
ros2 topic list | rg "scan|odom|cmd_vel|tf"
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic hz /cmd_vel
ros2 topic echo /amcl_pose --once
```

## 6. 常见现象与定位顺序

现象 1：RViz 里有全局路径，但机器人不动。  
优先检查：
- `fishbot_diff_drive_controller` 是否成功激活。  
- `/cmd_vel` 是否有数据。  
- `map->odom` 和 `odom->base_footprint` 是否连续。  

现象 2：机器人在走廊里抖动、贴墙、反复恢复。  
优先检查：
- local costmap 是否出现噪声团块。  
- 激光更新率和时间戳是否稳定。  
- RPP 前视距离与速度上限是否匹配当前地图尺度。  

现象 3：重启仿真后出现 TF 时间戳过早。  
优先处理：
- 关闭所有相关终端后整套重启。  
- 确保全栈统一 `use_sim_time: true`。  

## 7. 当前工程里的一个注意点

`nav2_params.yaml` 中 `default_nav_to_pose_bt_xml` 目前写的是绝对路径：

```text
/home/guzhen/chapt8/chapt8_ws/install/fishbot_navigation2/share/fishbot_navigation2/behavior_trees/navigate_to_pose_stable.xml
```

这在你当前机器可用，但如果仓库换路径或换电脑，可能找不到该文件。  
若后续要跨机器复现，建议把这个路径改成可移植写法（比如在 launch 中按包路径拼接后传入）。

## 8. 扩展阅读

- 导航参数专题：`src/fishbot_navigation2/README.md`
