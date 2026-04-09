# 基准地图标注说明

本目录用于全局规划压力测试，提供“多环境”地图集与标签说明。

## 地图列表

| 地图 ID | 地图 YAML | Gazebo World | 环境标签 | 难度 | 主要压力点 |
| --- | --- | --- | --- | --- | --- |
| `warehouse_aisles_01` | `warehouse_aisles_01.yaml` | `warehouse_aisles_01.world` | 仓储货架窄通道 | 中等 | 中轴性、贴墙风险、恢复次数 |
| `office_rooms_01` | `office_rooms_01.yaml` | `office_rooms_01.world` | 办公室房间+门洞 | 中等 | 频繁转角、门洞通过稳定性 |
| `maze_zigzag_01` | `maze_zigzag_01.yaml` | `maze_zigzag_01.world` | 迷宫锯齿通道 | 困难 | 路径鲁棒性、卡死率 |
| `open_clutter_01` | `open_clutter_01.yaml` | `open_clutter_01.world` | 开阔区+离散障碍 | 中等 | 效率与安全距离折中 |
| `turtlebot3_world` | `turtlebot3_world.yaml` | `turtlebot3_world.world` | Nav2 参考混合布局 | 中等 | 与社区基线对比 |

## 对应目标集

每张图都有对应目标集文件（10点）：

- `planner_benchmark_goals_warehouse.yaml`
- `planner_benchmark_goals_office.yaml`
- `planner_benchmark_goals_maze.yaml`
- `planner_benchmark_goals_clutter.yaml`
- `planner_benchmark_goals_tb3_world.yaml`

## 来源说明

- `turtlebot3_world.*` 从 `/opt/ros/humble/share/nav2_bringup/maps/` 引入。
- 其余地图为本仓库基准测试地图，分辨率统一 `0.05 m/cell`。
- `*.world` 由 `fishbot_description/world/benchmark/generate_benchmark_worlds.py` 根据对应地图自动生成。

## 使用提示

成对切换 world + map 示例（推荐）：

```bash
ros2 launch fishbot_description gazebo_sim.launch.py \
  world:=/home/guzhen/chapt8/chapt8_ws/src/fishbot_description/world/benchmark/warehouse_aisles_01.world

ros2 launch fishbot_navigation2 navigation2.launch.py \
  map:=/home/guzhen/chapt8/chapt8_ws/src/fishbot_navigation2/maps/benchmark/warehouse_aisles_01.yaml \
  params_file:=/home/guzhen/chapt8/chapt8_ws/src/fishbot_navigation2/config/nav2_params_custom.yaml
```
ros2 launch fishbot_description gazebo_sim.launch.py \
  world:=/home/guzhen/chapt8/chapt8_ws/src/fishbot_description/world/custom_room.world
