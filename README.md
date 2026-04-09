# chapt8_ws 项目总览与完整使用手册

本仓库是一个基于 ROS 2 Humble + Gazebo + Nav2 的移动机器人导航工作区。  
它包含机器人建模、仿真启动、导航参数、自定义全局规划器、以及多环境评测脚本。

如果你只想快速跑起来，先看第 4 节和第 5 节。  
如果你要做论文实验，重点看第 6 节和第 7 节。

## 1. 项目目标

本项目当前主要服务两个方向：

1. 搭建可稳定复现的室内导航仿真链路。
2. 对比 `SmacPlanner2D`、`NavfnPlanner`、`CustomPlanner` 三种全局规划器表现。

其中 `CustomPlanner` 已实现“清障项 + 中轴项 + 转向项 + 自适应权重”的总代价模型，用于多环境实验。

## 2. 工作区结构

`src/` 下的主要包如下：

| 包名 | 作用 |
| --- | --- |
| `fishbot_description` | 机器人 URDF、Gazebo 插件、控制器参数、世界文件与仿真启动 |
| `fishbot_navigation2` | Nav2 启动、地图、参数、行为树、评测配置 |
| `nav2_custom_planner` | 自定义全局规划器插件（A* + 自适应代价） |
| `nav2_custom_controller` | 自定义控制器插件（预留/扩展） |
| `fishbot_application` | Python 应用脚本（初始化位姿、评测、汇总） |
| `fishbot_application_cpp` | C++ 应用示例 |
| `autopatrol_interfaces` | 巡检相关接口定义（srv） |
| `autopatrol_robot` | 巡检应用包（预留/实验） |

## 3. 导航链路（从目标到底盘）

```text
RViz 2D Goal Pose
  -> bt_navigator
  -> planner_server（Smac / Navfn / Custom）
  -> smoother_server
  -> controller_server（RPP）
  -> velocity_smoother
  -> /cmd_vel
  -> fishbot_diff_drive_controller
  -> Gazebo 机器人运动
  -> /odom + TF
```

并行定位链路：

```text
map_server -> amcl（scan + odom + tf）-> map->odom
robot_state_publisher -> base_footprint->base_link->sensor links
```

判断系统是否正常，核心看两条闭环：

- 定位闭环：`map -> odom -> base_footprint`
- 控制闭环：`goal -> path -> cmd_vel -> odom`

## 4. 快速启动（基础导航）

### 4.1 编译

```bash
cd /home/guzhen/chapt8/chapt8_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  fishbot_description fishbot_navigation2 nav2_custom_planner fishbot_application
source install/setup.bash
```

### 4.2 启动仿真

```bash
ros2 launch fishbot_description gazebo_sim.launch.py
```

说明：

- 默认 world：`fishbot_description/world/custom_room.world`
- 启动文件会自动加载机器人并按顺序激活控制器：
- `fishbot_joint_state_broadcaster`
- `fishbot_diff_drive_controller`

### 4.3 启动 Nav2

```bash
ros2 launch fishbot_navigation2 navigation2.launch.py
```

启动后可在 RViz 用 `2D Goal Pose` 下发目标点。

### 4.4 手动遥控（可选）

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
```

用于快速验证底盘控制链路是否正常。

## 5. 多环境场景切换

项目已内置 benchmark 地图与对应 world，目录如下：

- 地图：`src/fishbot_navigation2/maps/benchmark/`
- 世界：`src/fishbot_description/world/benchmark/`
- 标注：`src/fishbot_navigation2/maps/benchmark/MAP_LABELS.md`
- 索引：`src/fishbot_navigation2/config/benchmark_map_registry.yaml`

切换原则：

- `map_yaml`、`world_file`、`goals_yaml` 必须成对使用。

示例（仓储场景）：

```bash
WORLD_FILE=/home/guzhen/chapt8/chapt8_ws/src/fishbot_description/world/benchmark/warehouse_aisles_01.world
MAP_YAML=/home/guzhen/chapt8/chapt8_ws/src/fishbot_navigation2/maps/benchmark/warehouse_aisles_01.yaml
GOALS_YAML=/home/guzhen/chapt8/chapt8_ws/src/fishbot_navigation2/config/planner_benchmark_goals_warehouse.yaml
```

然后分别启动：

```bash
ros2 launch fishbot_description gazebo_sim.launch.py \
  world:=$WORLD_FILE \
  goals_yaml:=$GOALS_YAML \
  auto_spawn_from_goals:=true
```

```bash
ros2 launch fishbot_navigation2 navigation2.launch.py \
  map:=$MAP_YAML \
  params_file:=/home/guzhen/chapt8/chapt8_ws/src/fishbot_navigation2/config/nav2_params_custom.yaml
```

`auto_spawn_from_goals:=true` 时，会自动读取 `goals_yaml` 的 `G01` 作为出生点。

## 6. 三规划器对比评测

### 6.1 规划器参数文件

- Smac：`src/fishbot_navigation2/config/nav2_params_smac.yaml`
- Navfn：`src/fishbot_navigation2/config/nav2_params_navfn.yaml`
- Custom：`src/fishbot_navigation2/config/nav2_params_custom.yaml`

### 6.2 运行评测脚本

入口脚本：

- `ros2 run fishbot_application planner_benchmark`
- `ros2 run fishbot_application planner_benchmark_summary`

单算法示例：

```bash
ros2 run fishbot_application planner_benchmark \
  --goals-file $GOALS_YAML \
  --output-csv /home/guzhen/chapt8/chapt8_ws/src/fishbot_navigation2/config/planner_benchmark_results.csv \
  --planner-name CustomPlanner \
  --runs 3 \
  --timeout-s 180
```

三算法对比时建议：

1. 第一种算法不加 `--append`（覆盖旧结果）。
2. 后两种算法加 `--append`（追加写入同一个结果文件）。

### 6.3 汇总统计

```bash
ros2 run fishbot_application planner_benchmark_summary \
  --input-csv /home/guzhen/chapt8/chapt8_ws/src/fishbot_navigation2/config/planner_benchmark_results.csv \
  --output-csv /home/guzhen/chapt8/chapt8_ws/src/fishbot_navigation2/config/planner_benchmark_summary.csv \
  --output-md /home/guzhen/chapt8/chapt8_ws/src/fishbot_navigation2/config/planner_benchmark_summary.md \
  --sort-by success
```

## 7. 数据文件说明

输出文件位于 `src/fishbot_navigation2/config/`：

- `planner_benchmark_results.csv`：原始样本（每次目标执行一行）
- `planner_benchmark_summary.csv`：按算法聚合统计
- `planner_benchmark_summary.md`：Markdown 表格版结果
- `planner_benchmark_template.csv`：模板/手工记录辅助

`results.csv` 的关键列：

- `planner_name`
- `run_id`
- `goal_id`
- `success`
- `time_to_goal_s`
- `stuck_or_not`
- `wall_hugging_level`
- `recovery_count`
- `note`

常见误解：

- “为什么不是 10 行？”
- 因为总样本数 = `目标点数 × runs × 算法数`。
- 例如 `10` 点、`runs=3`、`3` 个算法，总行数是 `90`（不含表头）。

## 8. CustomPlanner 说明

`nav2_custom_planner` 中的 `CustomPlanner` 已在 A* 基础上加入：

1. 安全距离代价 `C_clear`
2. 中轴偏置代价 `C_middle`
3. 转向代价 `C_turn`
4. 自适应权重 `lambda_c / lambda_m / lambda_t`

并采用方向增强状态，减少“只按 2D 闭集导致的转向代价路径依赖问题”。

对应核心源码：

- `src/nav2_custom_planner/include/nav2_custom_planner/nav2_custom_planner.hpp`
- `src/nav2_custom_planner/src/nav2_custom_planner.cpp`

## 9. 常见问题排查

机器人不动：

- 检查控制器是否激活：`fishbot_diff_drive_controller`
- 检查 `/cmd_vel` 是否有数据
- 检查 `map -> odom -> base_footprint` 是否连续

开局倒地或卡在障碍里：

- 使用 `goals_yaml + auto_spawn_from_goals:=true`
- 或手动传 `spawn_x/spawn_y/spawn_yaw`

切图后表现异常：

- 多数是 `world/map/goals` 没有成对切换

日志快速筛查：

```bash
rg -n "Failed to make progress|PlannerException|A\\*|timeout|recovery|TF_OLD_DATA" ~/.ros/log -g "*.log"
```

## 10. 推荐阅读

- 导航与评测细节：`src/fishbot_navigation2/README.md`
- 地图标签：`src/fishbot_navigation2/maps/benchmark/MAP_LABELS.md`
- 地图索引：`src/fishbot_navigation2/config/benchmark_map_registry.yaml`
