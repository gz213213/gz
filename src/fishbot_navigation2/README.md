# fishbot_navigation2 导航说明（Smac2D + Smoother + RPP）

## 1. 架构
- 全局规划：`nav2_smac_planner/SmacPlanner2D`（2D A* + 代价惩罚）
- 路径平滑：`nav2_smoother::SimpleSmoother`（BT 中显式 `SmoothPath`）
- 局部控制：`nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`
- 行为树：`behavior_trees/navigate_to_pose_stable.xml`（低频重规划 + Plan->Smooth->Follow + 温和恢复）
- 底盘基座坐标：`base_footprint`

## 2. 这版重点解决的问题
### 2.1 长距离走廊稳定跟踪
- 全局路径由 `SmacPlanner2D` 生成，配合 `cost_travel_multiplier` 增强“远离高代价墙边”的倾向。
- BT 在每次重规划后显式执行 `SmoothPath`，减少折线导致的方向抖动。
- `RPP` 前视距离整体拉远（`lookahead_dist/min/max`），在走廊中更稳、更少左右摆。
- 线速度上限与期望速度下调（“先慢一点”），优先稳定通过窄长空间。

### 2.2 全局路径靠墙过近
- `SmacPlanner2D.cost_travel_multiplier` 提高后，规划更倾向长廊中轴。
- `SmoothPath` 启用碰撞检查，避免平滑结果贴近障碍。

### 2.3 动态障碍与噪声
- `RPP` 开启碰撞前瞻（`max_allowed_time_to_collision_up_to_carrot`）。
- local obstacle layer 关闭持久化（`observation_persistence: 0.0`），减少“幽灵障碍”。

## 3. 关键参数（当前）
| 模块 | 参数 | 值 |
| --- | --- | --- |
| bt_navigator | `default_nav_to_pose_bt_xml` | `src/fishbot_navigation2/behavior_trees/navigate_to_pose_stable.xml` |
| controller_server | `controller_frequency` | `12.0` |
| controller_server.FollowPath | `plugin` | `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` |
| controller_server.FollowPath | `desired_linear_vel` | `0.10` |
| controller_server.FollowPath | `lookahead_dist / min / max` | `0.45 / 0.35 / 0.90` |
| controller_server.FollowPath | `lookahead_time` | `1.8` |
| controller_server.progress_checker | `movement_time_allowance` | `35.0` |
| planner_server.GridBased | `plugin` | `nav2_smac_planner/SmacPlanner2D` |
| planner_server.GridBased | `cost_travel_multiplier` | `3.0` |
| smoother_server.simple_smoother | `w_data / w_smooth` | `0.2 / 0.35` |
| local_costmap.scan | `observation_persistence` | `0.0` |

## 4. 启动
```bash
cd /home/guzhen/chapt8/chapt8_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select fishbot_navigation2 fishbot_description
source install/setup.bash
ros2 launch fishbot_description gazebo_sim.launch.py
```

新开终端：
```bash
cd /home/guzhen/chapt8/chapt8_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch fishbot_navigation2 navigation2.launch.py
```

## 5. 终端排障（优先看）
```bash
rg -n "Failed to make progress|Control loop missed|tick rate|Timed out while waiting for action server" ~/.ros/log -g "*.log"
```

- 若主要是 `Failed to make progress`：先看 local costmap 是否有噪声团块/幽灵障碍。
- 若主要是 `Control loop missed` + `tick rate exceeded`：先确认是否已加载 RPP 与稳定版 BT。
- 若 RViz 有 `Message Filter dropping message` 且提到 timestamp 过早：通常是仿真重启后的时钟/TF缓存问题，建议整套节点重启并确保 `use_sim_time=true`。
