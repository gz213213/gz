#include "nav2_custom_planner/nav2_custom_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_custom_planner
{
namespace
{
struct OpenNode
{
  size_t index;
  double f;
  double g;
};

struct CompareOpenNode
{
  bool operator()(const OpenNode & a, const OpenNode & b) const
  {
    return a.f > b.f;
  }
};

using GridPoint = std::pair<unsigned int, unsigned int>;

constexpr std::array<int, 8> kDx{{-1, -1, -1, 0, 0, 1, 1, 1}};
constexpr std::array<int, 8> kDy{{-1, 0, 1, -1, 1, -1, 0, 1}};
constexpr double kDiagonalCost = 1.41421356237;
constexpr std::array<double, 8> kMoveCost{
  {kDiagonalCost, 1.0, kDiagonalCost, 1.0, 1.0, kDiagonalCost, 1.0, kDiagonalCost}};

inline size_t toIndex(unsigned int x, unsigned int y, unsigned int width)
{
  return static_cast<size_t>(y) * width + x;
}

inline GridPoint toCell(size_t index, unsigned int width)
{
  return {static_cast<unsigned int>(index % width), static_cast<unsigned int>(index / width)};
}

inline bool inBounds(int x, int y, unsigned int width, unsigned int height)
{
  return x >= 0 && y >= 0 && static_cast<unsigned int>(x) < width && static_cast<unsigned int>(y) < height;
}

double heuristic(
  unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2, bool use_manhattan_distance)
{
  if (use_manhattan_distance) {
    return std::abs(static_cast<int>(x1) - static_cast<int>(x2)) +
           std::abs(static_cast<int>(y1) - static_cast<int>(y2));
  }
  return std::hypot(
    static_cast<double>(static_cast<int>(x1) - static_cast<int>(x2)),
    static_cast<double>(static_cast<int>(y1) - static_cast<int>(y2)));
}

bool isCellTraversable(
  nav2_costmap_2d::Costmap2D * costmap, unsigned int x, unsigned int y, bool allow_unknown,
  int lethal_cost_threshold)
{
  const unsigned char cost = costmap->getCost(x, y);
  if (cost == nav2_costmap_2d::NO_INFORMATION) {
    return allow_unknown;
  }
  return static_cast<int>(cost) < lethal_cost_threshold;
}

bool canTraverseStep(
  nav2_costmap_2d::Costmap2D * costmap, unsigned int from_x, unsigned int from_y, unsigned int to_x,
  unsigned int to_y, bool allow_unknown, bool allow_corner_cutting, int lethal_cost_threshold)
{
  if (!isCellTraversable(costmap, to_x, to_y, allow_unknown, lethal_cost_threshold)) {
    return false;
  }

  const int dx = static_cast<int>(to_x) - static_cast<int>(from_x);
  const int dy = static_cast<int>(to_y) - static_cast<int>(from_y);
  const bool diagonal_move = std::abs(dx) == 1 && std::abs(dy) == 1;
  if (!diagonal_move || allow_corner_cutting) {
    return true;
  }

  // 防穿角：禁止在两侧正交格子被占据时进行对角扩展。
  const unsigned int side_x = static_cast<unsigned int>(static_cast<int>(from_x) + dx);
  const unsigned int side_y = from_y;
  const unsigned int side2_x = from_x;
  const unsigned int side2_y = static_cast<unsigned int>(static_cast<int>(from_y) + dy);

  return isCellTraversable(costmap, side_x, side_y, allow_unknown, lethal_cost_threshold) &&
         isCellTraversable(costmap, side2_x, side2_y, allow_unknown, lethal_cost_threshold);
}

std::vector<GridPoint> reconstructPath(
  const std::vector<int64_t> & came_from, size_t start_index, size_t goal_index, unsigned int width)
{
  std::vector<GridPoint> path;
  size_t current = goal_index;

  while (true) {
    path.push_back(toCell(current, width));
    if (current == start_index) {
      break;
    }
    if (came_from[current] < 0) {
      return {};
    }
    current = static_cast<size_t>(came_from[current]);
  }

  std::reverse(path.begin(), path.end());
  return path;
}

bool hasLineOfSight(
  nav2_costmap_2d::Costmap2D * costmap, const GridPoint & from, const GridPoint & to, bool allow_unknown,
  bool allow_corner_cutting, int lethal_cost_threshold, int smoothing_max_cost)
{
  int x0 = static_cast<int>(from.first);
  int y0 = static_cast<int>(from.second);
  const int x1 = static_cast<int>(to.first);
  const int y1 = static_cast<int>(to.second);

  const int dx = std::abs(x1 - x0);
  const int dy = std::abs(y1 - y0);
  const int sx = x0 < x1 ? 1 : -1;
  const int sy = y0 < y1 ? 1 : -1;
  int err = dx - dy;

  while (true) {
    const auto cx = static_cast<unsigned int>(x0);
    const auto cy = static_cast<unsigned int>(y0);
    if (!isCellTraversable(costmap, cx, cy, allow_unknown, lethal_cost_threshold)) {
      return false;
    }

    // 平滑阶段附加安全约束：高代价区域不允许直连，防止路径被“抹直”到墙边。
    if (static_cast<int>(costmap->getCost(cx, cy)) >= smoothing_max_cost) {
      return false;
    }

    if (x0 == x1 && y0 == y1) {
      return true;
    }

    const int prev_x = x0;
    const int prev_y = y0;
    const int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x0 += sx;
    }
    if (e2 < dx) {
      err += dx;
      y0 += sy;
    }

    if (!canTraverseStep(
          costmap, static_cast<unsigned int>(prev_x), static_cast<unsigned int>(prev_y),
          static_cast<unsigned int>(x0), static_cast<unsigned int>(y0), allow_unknown,
          allow_corner_cutting, lethal_cost_threshold))
    {
      return false;
    }
  }
}

std::vector<GridPoint> smoothPathByLineOfSight(
  nav2_costmap_2d::Costmap2D * costmap, const std::vector<GridPoint> & raw_path, bool allow_unknown,
  bool allow_corner_cutting, int lethal_cost_threshold, int smoothing_max_cost)
{
  if (raw_path.size() <= 2) {
    return raw_path;
  }

  std::vector<GridPoint> smoothed_path;
  smoothed_path.reserve(raw_path.size());
  smoothed_path.push_back(raw_path.front());

  size_t anchor = 0;
  while (anchor < raw_path.size() - 1) {
    size_t farthest_reachable = anchor + 1;

    // 从 anchor 往后找可直连的最远节点，减少不必要折线并保持障碍约束。
    for (size_t candidate = anchor + 1; candidate < raw_path.size(); ++candidate) {
      if (!hasLineOfSight(
            costmap, raw_path[anchor], raw_path[candidate], allow_unknown, allow_corner_cutting,
            lethal_cost_threshold, smoothing_max_cost))
      {
        break;
      }
      farthest_reachable = candidate;
    }

    smoothed_path.push_back(raw_path[farthest_reachable]);
    anchor = farthest_reachable;
  }

  return smoothed_path;
}

std::vector<geometry_msgs::msg::PoseStamped> interpolatePath(
  const std::vector<geometry_msgs::msg::PoseStamped> & coarse_path, double interpolation_resolution)
{
  if (coarse_path.size() < 2 || interpolation_resolution <= 0.0) {
    return coarse_path;
  }

  std::vector<geometry_msgs::msg::PoseStamped> dense_path;
  dense_path.reserve(coarse_path.size() * 2);
  dense_path.push_back(coarse_path.front());

  for (size_t i = 0; i + 1 < coarse_path.size(); ++i) {
    const auto & from = coarse_path[i];
    const auto & to = coarse_path[i + 1];

    const double dx = to.pose.position.x - from.pose.position.x;
    const double dy = to.pose.position.y - from.pose.position.y;
    const double segment_length = std::hypot(dx, dy);
    const int steps = std::max(1, static_cast<int>(std::ceil(segment_length / interpolation_resolution)));

    for (int step = 1; step <= steps; ++step) {
      const double ratio = static_cast<double>(step) / static_cast<double>(steps);
      geometry_msgs::msg::PoseStamped interpolated = from;
      interpolated.pose.position.x = from.pose.position.x + ratio * dx;
      interpolated.pose.position.y = from.pose.position.y + ratio * dy;
      dense_path.push_back(interpolated);
    }
  }

  return dense_path;
}

void assignPoseOrientation(
  std::vector<geometry_msgs::msg::PoseStamped> & poses, const geometry_msgs::msg::Quaternion & goal_orientation)
{
  if (poses.empty()) {
    return;
  }

  for (size_t i = 0; i + 1 < poses.size(); ++i) {
    const double dx = poses[i + 1].pose.position.x - poses[i].pose.position.x;
    const double dy = poses[i + 1].pose.position.y - poses[i].pose.position.y;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, std::atan2(dy, dx));
    poses[i].pose.orientation = tf2::toMsg(q);
  }

  poses.back().pose.orientation = goal_orientation;
}
}  // namespace

void CustomPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  tf_ = tf;
  node_ = parent.lock();
  name_ = name;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".use_manhattan_distance", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".enable_path_smoothing", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".max_search_iterations", rclcpp::ParameterValue(200000));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".allow_unknown", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".allow_corner_cutting", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".lethal_cost_threshold",
    rclcpp::ParameterValue(static_cast<int>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".obstacle_cost_weight", rclcpp::ParameterValue(3.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".smoothing_max_cost", rclcpp::ParameterValue(190));

  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
  node_->get_parameter(name_ + ".use_manhattan_distance", use_manhattan_distance_);
  node_->get_parameter(name_ + ".enable_path_smoothing", enable_path_smoothing_);
  node_->get_parameter(name_ + ".max_search_iterations", max_search_iterations_);
  node_->get_parameter(name_ + ".allow_unknown", allow_unknown_);
  node_->get_parameter(name_ + ".allow_corner_cutting", allow_corner_cutting_);
  node_->get_parameter(name_ + ".lethal_cost_threshold", lethal_cost_threshold_);
  node_->get_parameter(name_ + ".obstacle_cost_weight", obstacle_cost_weight_);
  node_->get_parameter(name_ + ".smoothing_max_cost", smoothing_max_cost_);

  lethal_cost_threshold_ = std::max(1, std::min(lethal_cost_threshold_, 255));
  smoothing_max_cost_ = std::max(1, std::min(smoothing_max_cost_, lethal_cost_threshold_));
  obstacle_cost_weight_ = std::max(0.0, obstacle_cost_weight_);
}

void CustomPlanner::cleanup()
{
  RCLCPP_INFO(node_->get_logger(), "正在清理类型为 CustomPlanner 的插件 %s", name_.c_str());
}

void CustomPlanner::activate()
{
  RCLCPP_INFO(node_->get_logger(), "正在激活类型为 CustomPlanner 的插件 %s", name_.c_str());
}

void CustomPlanner::deactivate()
{
  RCLCPP_INFO(node_->get_logger(), "正在停用类型为 CustomPlanner 的插件 %s", name_.c_str());
}

nav_msgs::msg::Path CustomPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path_msg;
  path_msg.header.stamp = node_->now();
  path_msg.header.frame_id = global_frame_;

  if (start.header.frame_id != global_frame_) {
    throw nav2_core::PlannerException("起点坐标系不匹配，期望: " + global_frame_);
  }
  if (goal.header.frame_id != global_frame_) {
    throw nav2_core::PlannerException("终点坐标系不匹配，期望: " + global_frame_);
  }

  unsigned int start_x = 0;
  unsigned int start_y = 0;
  unsigned int goal_x = 0;
  unsigned int goal_y = 0;
  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y)) {
    throw nav2_core::PlannerException("起点不在代价地图范围内");
  }
  if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y)) {
    throw nav2_core::PlannerException("终点不在代价地图范围内");
  }

  if (!isCellTraversable(costmap_, start_x, start_y, allow_unknown_, lethal_cost_threshold_)) {
    throw nav2_core::PlannerException("起点位于障碍或膨胀危险区内，无法规划");
  }
  if (!isCellTraversable(costmap_, goal_x, goal_y, allow_unknown_, lethal_cost_threshold_)) {
    throw nav2_core::PlannerException("终点位于障碍或膨胀危险区内，无法规划");
  }

  const unsigned int width = costmap_->getSizeInCellsX();
  const unsigned int height = costmap_->getSizeInCellsY();
  const size_t cell_count = static_cast<size_t>(width) * height;
  const size_t start_index = toIndex(start_x, start_y, width);
  const size_t goal_index = toIndex(goal_x, goal_y, width);

  std::vector<double> g_score(cell_count, std::numeric_limits<double>::infinity());
  std::vector<int64_t> came_from(cell_count, -1);
  std::vector<bool> closed(cell_count, false);
  std::priority_queue<OpenNode, std::vector<OpenNode>, CompareOpenNode> open_set;

  g_score[start_index] = 0.0;
  open_set.push(
    {start_index, heuristic(start_x, start_y, goal_x, goal_y, use_manhattan_distance_), 0.0});

  bool goal_reached = false;
  int search_iterations = 0;

  // A* 主循环：每次从 open_set 取 f 最小节点扩展，直到到达目标或超出限制。
  while (!open_set.empty()) {
    const OpenNode current = open_set.top();
    open_set.pop();

    if (current.g > g_score[current.index]) {
      continue;
    }
    if (closed[current.index]) {
      continue;
    }
    closed[current.index] = true;

    if (current.index == goal_index) {
      goal_reached = true;
      break;
    }

    if (++search_iterations > max_search_iterations_) {
      throw nav2_core::PlannerException("A* 搜索超过最大迭代次数，终止规划");
    }

    const GridPoint current_cell = toCell(current.index, width);
    const unsigned int cx = current_cell.first;
    const unsigned int cy = current_cell.second;
    for (size_t dir = 0; dir < kDx.size(); ++dir) {
      const int nx = static_cast<int>(cx) + kDx[dir];
      const int ny = static_cast<int>(cy) + kDy[dir];

      if (!inBounds(nx, ny, width, height)) {
        continue;
      }

      const auto ux = static_cast<unsigned int>(nx);
      const auto uy = static_cast<unsigned int>(ny);
      if (!canTraverseStep(
            costmap_, cx, cy, ux, uy, allow_unknown_, allow_corner_cutting_,
            lethal_cost_threshold_))
      {
        continue;
      }

      const size_t neighbor_index = toIndex(ux, uy, width);
      if (closed[neighbor_index]) {
        continue;
      }

      // 将代价地图代价纳入 g 值，鼓励路径远离高代价区域（贴墙和窄缝更少）。
      const unsigned char cell_cost = costmap_->getCost(ux, uy);
      const double normalized_penalty =
        static_cast<double>(std::min<int>(cell_cost, lethal_cost_threshold_ - 1)) /
        static_cast<double>(lethal_cost_threshold_);
      // 二次惩罚比线性惩罚更偏好通道中轴，能显著降低“贴墙走”的概率。
      const double weighted_penalty =
        obstacle_cost_weight_ * normalized_penalty * normalized_penalty;
      const double tentative_g = g_score[current.index] + kMoveCost[dir] + weighted_penalty;

      if (tentative_g < g_score[neighbor_index]) {
        came_from[neighbor_index] = static_cast<int64_t>(current.index);
        g_score[neighbor_index] = tentative_g;
        const double h = heuristic(ux, uy, goal_x, goal_y, use_manhattan_distance_);
        open_set.push({neighbor_index, tentative_g + h, tentative_g});
      }
    }
  }

  if (!goal_reached) {
    throw nav2_core::PlannerException(
            "A* 无法找到可行路径，目标坐标: (" + std::to_string(goal.pose.position.x) + ", " +
            std::to_string(goal.pose.position.y) + ")");
  }

  std::vector<GridPoint> cell_path = reconstructPath(came_from, start_index, goal_index, width);
  if (cell_path.empty()) {
    throw nav2_core::PlannerException("A* 回溯失败，路径为空");
  }

  if (enable_path_smoothing_) {
    // 只在保持可通行约束下做直线化平滑，避免把路径“抹”进障碍物。
    cell_path = smoothPathByLineOfSight(
      costmap_, cell_path, allow_unknown_, allow_corner_cutting_, lethal_cost_threshold_,
      smoothing_max_cost_);
  }

  std::vector<geometry_msgs::msg::PoseStamped> coarse_path;
  coarse_path.reserve(cell_path.size());
  for (const auto & cell : cell_path) {
    const unsigned int cell_x = cell.first;
    const unsigned int cell_y = cell.second;
    double wx = 0.0;
    double wy = 0.0;
    costmap_->mapToWorld(cell_x, cell_y, wx, wy);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path_msg.header;
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.0;
    coarse_path.push_back(pose);
  }

  if (coarse_path.empty()) {
    throw nav2_core::PlannerException("路径转换为世界坐标失败");
  }

  const double goal_delta = std::hypot(
    coarse_path.back().pose.position.x - goal.pose.position.x,
    coarse_path.back().pose.position.y - goal.pose.position.y);
  if (goal_delta > 0.01) {
    geometry_msgs::msg::PoseStamped goal_pose = goal;
    goal_pose.header = path_msg.header;
    coarse_path.push_back(goal_pose);
  } else {
    coarse_path.back().pose.position.x = goal.pose.position.x;
    coarse_path.back().pose.position.y = goal.pose.position.y;
    coarse_path.back().pose.position.z = goal.pose.position.z;
  }

  auto dense_path = interpolatePath(coarse_path, interpolation_resolution_);
  assignPoseOrientation(dense_path, goal.pose.orientation);
  path_msg.poses = std::move(dense_path);
  return path_msg;
}

}  // namespace nav2_custom_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_custom_planner::CustomPlanner, nav2_core::GlobalPlanner)
