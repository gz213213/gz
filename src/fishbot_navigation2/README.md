# fishbot_navigation2 实验与评测手册（完整流程版）

本文档用于当前仓库的 Nav2 仿真、三规划器对比评测与论文数据整理。  
目标是做到“按文档一步步执行，就能稳定复现实验结果”。

## 1. 你会用到的核心文件

| 类别 | 文件 |
| --- | --- |
| 规划器参数 | `config/nav2_params_smac.yaml` |
| 规划器参数 | `config/nav2_params_navfn.yaml` |
| 规划器参数 | `config/nav2_params_custom.yaml` |
| 启动入口 | `launch/navigation2.launch.py` |
| 多环境地图目录 | `maps/benchmark/` |
| 地图标签说明 | `maps/benchmark/MAP_LABELS.md` |
| 地图-世界-目标集索引 | `config/benchmark_map_registry.yaml` |
| 目标点文件（默认） | `config/planner_benchmark_goals.yaml` |
| 目标点文件（多环境） | `config/planner_benchmark_goals_*.yaml` |
| 原始结果 | `config/planner_benchmark_results.csv` |
| 汇总结果 | `config/planner_benchmark_summary.csv` |
| 汇总结果（Markdown） | `config/planner_benchmark_summary.md` |

## 2. 一次性准备

```bash
cd /home/guzhen/chapt8/chapt8_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  fishbot_description fishbot_navigation2 nav2_custom_planner fishbot_application
source install/setup.bash
```

## 3. 先选场景（必须成对）

推荐从 `config/benchmark_map_registry.yaml` 里选同一个 `map_id` 的三项：

- `world_file`
- `map_yaml`
- `goals_yaml`

示例（仓储场景）：

```bash
WORLD_FILE=/home/guzhen/chapt8/chapt8_ws/src/fishbot_description/world/benchmark/warehouse_aisles_01.world
MAP_YAML=/home/guzhen/chapt8/chapt8_ws/src/fishbot_navigation2/maps/benchmark/warehouse_aisles_01.yaml
GOALS_YAML=/home/guzhen/chapt8/chapt8_ws/src/fishbot_navigation2/config/planner_benchmark_goals_warehouse.yaml
```

注意：

- `world` 和 `map` 不成对时，常见现象是定位漂移、路径异常、恢复行为变多。
- `gazebo_sim.launch.py` 支持从 `goals_yaml` 自动读取 `G01` 作为出生点。

## 4. 启动仿真与导航

终端 1：Gazebo

```bash
cd /home/guzhen/chapt8/chapt8_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch fishbot_description gazebo_sim.launch.py \
  world:=$WORLD_FILE \
  goals_yaml:=$GOALS_YAML \
  auto_spawn_from_goals:=true
```

终端 2：Nav2（每次只跑一种规划器）

```bash
# SmacPlanner2D
ros2 launch fishbot_navigation2 navigation2.launch.py \
  map:=$MAP_YAML \
  params_file:=/home/guzhen/chapt8/chapt8_ws/src/fishbot_navigation2/config/nav2_params_smac.yaml

# NavfnPlanner
ros2 launch fishbot_navigation2 navigation2.launch.py \
  map:=$MAP_YAML \
  params_file:=/home/guzhen/chapt8/chapt8_ws/src/fishbot_navigation2/config/nav2_params_navfn.yaml

# CustomPlanner
ros2 launch fishbot_navigation2 navigation2.launch.py \
  map:=$MAP_YAML \
  params_file:=/home/guzhen/chapt8/chapt8_ws/src/fishbot_navigation2/config/nav2_params_custom.yaml
```

## 5. 手动控制与功能验证

键盘遥控（验证底盘链路是否通）：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
```

导航验证：

- 打开 RViz 后使用 `2D Goal Pose` 下发目标点。
- 若能稳定运动，说明 `map -> odom -> base_footprint` 与控制链路基本正常。

## 6. 标准评测流程（单算法）

终端 3：

```bash
source /opt/ros/humble/setup.bash
source /home/guzhen/chapt8/chapt8_ws/install/setup.bash

ros2 run fishbot_application planner_benchmark \
  --goals-file $GOALS_YAML \
  --output-csv /home/guzhen/chapt8/chapt8_ws/src/fishbot_navigation2/config/planner_benchmark_results.csv \
  --planner-name CustomPlanner \
  --runs 3 \
  --timeout-s 180
```

参数说明：

- `--runs 3` 表示每个目标点重复 3 轮。
- 单算法总样本数 = `目标点数量 × runs`。
- `10` 个点、`runs=3` 时，单算法应写入 `30` 行数据（不含表头）。

## 7. 三算法对比流程（推荐）

执行顺序：

1. 启动 Gazebo（固定同一世界和出生策略）。
2. 启动 Nav2 + `Smac` 参数，跑评测。
3. 关闭 Nav2，改 `Navfn` 参数，跑评测。
4. 关闭 Nav2，改 `Custom` 参数，跑评测。
5. 生成汇总统计。

数据写入规则：

- 第一个算法不加 `--append`，默认覆盖写。
- 第二个和第三个算法加 `--append`，追加写入同一个结果文件。

示例：

```bash
# 1) Smac（覆盖写）
ros2 run fishbot_application planner_benchmark \
  --goals-file $GOALS_YAML \
  --output-csv /home/guzhen/chapt8/chapt8_ws/src/fishbot_navigation2/config/planner_benchmark_results.csv \
  --planner-name SmacPlanner2D \
  --runs 3 \
  --timeout-s 180

# 2) Navfn（追加写）
ros2 run fishbot_application planner_benchmark \
  --goals-file $GOALS_YAML \
  --output-csv /home/guzhen/chapt8/chapt8_ws/src/fishbot_navigation2/config/planner_benchmark_results.csv \
  --planner-name NavfnPlanner \
  --runs 3 \
  --timeout-s 180 \
  --append

# 3) Custom（追加写）
ros2 run fishbot_application planner_benchmark \
  --goals-file $GOALS_YAML \
  --output-csv /home/guzhen/chapt8/chapt8_ws/src/fishbot_navigation2/config/planner_benchmark_results.csv \
  --planner-name CustomPlanner \
  --runs 3 \
  --timeout-s 180 \
  --append
```

## 8. 一键统计汇总

```bash
source /opt/ros/humble/setup.bash
source /home/guzhen/chapt8/chapt8_ws/install/setup.bash

ros2 run fishbot_application planner_benchmark_summary \
  --input-csv /home/guzhen/chapt8/chapt8_ws/src/fishbot_navigation2/config/planner_benchmark_results.csv \
  --output-csv /home/guzhen/chapt8/chapt8_ws/src/fishbot_navigation2/config/planner_benchmark_summary.csv \
  --output-md /home/guzhen/chapt8/chapt8_ws/src/fishbot_navigation2/config/planner_benchmark_summary.md \
  --sort-by success
```

支持排序：

- `--sort-by success`
- `--sort-by time`
- `--sort-by stuck`
- `--sort-by planner`

## 9. 结果文件字段解释

`planner_benchmark_results.csv`：

- `planner_name`：算法名。
- `run_id`：第几轮重复实验。
- `goal_id`：目标点编号（如 `G01`）。
- `success`：是否成功到达（`1/0`）。
- `time_to_goal_s`：耗时（秒）。
- `stuck_or_not`：是否卡死或超时（`1/0`）。
- `wall_hugging_level`：贴墙程度（可人工补打 `1~5`）。
- `recovery_count`：恢复行为次数（脚本自动记录）。
- `note`：`ok/failed/canceled/timeout>...`。

`planner_benchmark_summary.csv`：

- `success_rate_pct`：成功率（%）。
- `stuck_rate_pct`：卡死率（%）。
- `avg_time_success_s`：成功样本平均耗时。
- `avg_recovery_count`：平均恢复次数。
- `avg_wall_hugging_level`：平均贴墙评分（只统计可解析数字的样本）。

## 10. 数据重置与覆盖规则

- 默认行为：不加 `--append` 会覆盖 `planner_benchmark_results.csv`。
- 追加行为：加 `--append` 会在原文件后继续写，不会清空旧数据。

你看到“超过 10 个点”的常见原因：

1. `runs > 1`（例如 `runs=3` 时，10 点会变成 30 行）。
2. 同一个结果文件累计了多个算法结果。
3. 多次执行时使用了 `--append`。

清空旧结果最稳妥方式：

```bash
rm -f /home/guzhen/chapt8/chapt8_ws/src/fishbot_navigation2/config/planner_benchmark_results.csv \
      /home/guzhen/chapt8/chapt8_ws/src/fishbot_navigation2/config/planner_benchmark_summary.csv \
      /home/guzhen/chapt8/chapt8_ws/src/fishbot_navigation2/config/planner_benchmark_summary.md
```

## 11. 论文建议指标与图表

主表建议：

1. 成功率（`success_rate_pct`）。
2. 卡死率（`stuck_rate_pct`）。
3. 成功样本平均耗时（`avg_time_success_s`）。
4. 平均恢复次数（`avg_recovery_count`）。

推荐图表：

1. 三算法成功率柱状图（分地图）。
2. 成功耗时箱线图（分算法）。
3. 恢复次数柱状图（分地图）。
4. 典型路径叠加图（同一 `goal_id` 下三算法轨迹）。

## 12. 常见问题排查

快速查日志：

```bash
rg -n "Failed to make progress|PlannerException|A\\*|timeout|recovery|TF_OLD_DATA" ~/.ros/log -g "*.log"
```

机器人不动：

- 检查 `fishbot_diff_drive_controller` 是否 `active`。
- 检查 `/cmd_vel` 是否有数据。
- 检查 TF 是否连续：`map -> odom -> base_footprint`。

开局倒地或卡住：

- 优先使用 `goals_yaml + auto_spawn_from_goals:=true`，由 `G01` 自动出生。
- 或手动设置 `spawn_x/spawn_y/spawn_yaw` 到空旷位置。

路径明显不合理：

- 检查是否误用不成对的 `world/map/goals`。
- 确认当前 `params_file` 与你要测试的规划器一致。
