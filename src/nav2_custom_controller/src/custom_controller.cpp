#include "nav2_custom_controller/custom_controller.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>

#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "tf2/utils.h"

namespace nav2_custom_controller
{

double CustomController::normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double CustomController::clamp(double value, double low, double high)
{
  return std::max(low, std::min(value, high));
}

void CustomController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  if (!node_) {
    throw std::runtime_error("CustomController 无法获取生命周期节点");
  }

  plugin_name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  costmap_ = costmap_ros_->getCostmap();

  // 速度与加速度约束：直接影响“能不能动起来”和“会不会抖动”。
  nav2_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".max_linear_speed", rclcpp::ParameterValue(max_linear_speed_));
  nav2_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".min_linear_speed", rclcpp::ParameterValue(min_linear_speed_));
  nav2_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".max_angular_speed", rclcpp::ParameterValue(max_angular_speed_));
  nav2_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".max_linear_accel", rclcpp::ParameterValue(max_linear_accel_));
  nav2_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".max_angular_accel", rclcpp::ParameterValue(max_angular_accel_));

  // 路径跟踪参数：前视距离、横向误差增益、原地转向触发阈值。
  nav2_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".min_lookahead_dist", rclcpp::ParameterValue(min_lookahead_dist_));
  nav2_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".max_lookahead_dist", rclcpp::ParameterValue(max_lookahead_dist_));
  nav2_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".lookahead_time", rclcpp::ParameterValue(lookahead_time_));
  nav2_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".heading_kp", rclcpp::ParameterValue(heading_kp_));
  nav2_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".lateral_kp", rclcpp::ParameterValue(lateral_kp_));
  nav2_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".rotate_in_place_min_angle",
    rclcpp::ParameterValue(rotate_in_place_min_angle_));

  // 脱困参数：用于“贴墙卡住”时主动旋转+小步前进，避免无穷等待 progress checker。
  nav2_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".forward_check_dist", rclcpp::ParameterValue(forward_check_dist_));
  nav2_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".forward_check_step", rclcpp::ParameterValue(forward_check_step_));
  nav2_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".side_probe_dist", rclcpp::ParameterValue(side_probe_dist_));
  nav2_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".obstacle_cost_threshold",
    rclcpp::ParameterValue(obstacle_cost_threshold_));
  nav2_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".escape_angular_speed", rclcpp::ParameterValue(escape_angular_speed_));
  nav2_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".escape_linear_speed", rclcpp::ParameterValue(escape_linear_speed_));

  // 输出平滑参数：缓解 VM + Gazebo 条件下速度命令离散导致的抖动。
  nav2_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".linear_velocity_smoothing",
    rclcpp::ParameterValue(linear_velocity_smoothing_));
  nav2_util::declare_parameter_if_not_declared(
    node_, plugin_name_ + ".angular_velocity_smoothing",
    rclcpp::ParameterValue(angular_velocity_smoothing_));

  node_->get_parameter(plugin_name_ + ".max_linear_speed", max_linear_speed_);
  node_->get_parameter(plugin_name_ + ".min_linear_speed", min_linear_speed_);
  node_->get_parameter(plugin_name_ + ".max_angular_speed", max_angular_speed_);
  node_->get_parameter(plugin_name_ + ".max_linear_accel", max_linear_accel_);
  node_->get_parameter(plugin_name_ + ".max_angular_accel", max_angular_accel_);

  node_->get_parameter(plugin_name_ + ".min_lookahead_dist", min_lookahead_dist_);
  node_->get_parameter(plugin_name_ + ".max_lookahead_dist", max_lookahead_dist_);
  node_->get_parameter(plugin_name_ + ".lookahead_time", lookahead_time_);
  node_->get_parameter(plugin_name_ + ".heading_kp", heading_kp_);
  node_->get_parameter(plugin_name_ + ".lateral_kp", lateral_kp_);
  node_->get_parameter(plugin_name_ + ".rotate_in_place_min_angle", rotate_in_place_min_angle_);

  node_->get_parameter(plugin_name_ + ".forward_check_dist", forward_check_dist_);
  node_->get_parameter(plugin_name_ + ".forward_check_step", forward_check_step_);
  node_->get_parameter(plugin_name_ + ".side_probe_dist", side_probe_dist_);
  node_->get_parameter(plugin_name_ + ".obstacle_cost_threshold", obstacle_cost_threshold_);
  node_->get_parameter(plugin_name_ + ".escape_angular_speed", escape_angular_speed_);
  node_->get_parameter(plugin_name_ + ".escape_linear_speed", escape_linear_speed_);

  node_->get_parameter(plugin_name_ + ".linear_velocity_smoothing", linear_velocity_smoothing_);
  node_->get_parameter(plugin_name_ + ".angular_velocity_smoothing", angular_velocity_smoothing_);

  max_linear_speed_ = std::max(0.01, max_linear_speed_);
  min_linear_speed_ = clamp(min_linear_speed_, 0.0, max_linear_speed_);
  max_angular_speed_ = std::max(0.1, max_angular_speed_);
  max_linear_accel_ = std::max(0.05, max_linear_accel_);
  max_angular_accel_ = std::max(0.1, max_angular_accel_);
  min_lookahead_dist_ = std::max(0.05, min_lookahead_dist_);
  max_lookahead_dist_ = std::max(min_lookahead_dist_, max_lookahead_dist_);
  lookahead_time_ = std::max(0.1, lookahead_time_);
  obstacle_cost_threshold_ = std::max(1, std::min(obstacle_cost_threshold_, 254));
  linear_velocity_smoothing_ = clamp(linear_velocity_smoothing_, 0.0, 0.98);
  angular_velocity_smoothing_ = clamp(angular_velocity_smoothing_, 0.0, 0.98);

  last_cmd_.twist.linear.x = 0.0;
  last_cmd_.twist.angular.z = 0.0;
  last_cmd_time_ = node_->now();

  RCLCPP_INFO(
    node_->get_logger(),
    "CustomController 已配置: v_max=%.3f, w_max=%.3f, obstacle_cost_threshold=%d",
    max_linear_speed_, max_angular_speed_, obstacle_cost_threshold_);
}

void CustomController::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "清理控制器插件: %s (nav2_custom_controller::CustomController)",
    plugin_name_.c_str());
}

void CustomController::activate()
{
  RCLCPP_INFO(node_->get_logger(), "激活控制器插件: %s", plugin_name_.c_str());
}

void CustomController::deactivate()
{
  RCLCPP_INFO(node_->get_logger(), "停用控制器插件: %s", plugin_name_.c_str());
}

geometry_msgs::msg::TwistStamped CustomController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  nav_msgs::msg::Path local_plan;
  {
    std::lock_guard<std::mutex> lock(plan_mutex_);
    local_plan = global_plan_;
  }

  if (local_plan.poses.empty()) {
    throw nav2_core::PlannerException("CustomController 收到空路径，无法计算速度");
  }

  geometry_msgs::msg::PoseStamped pose_in_plan;
  if (!nav2_util::transformPoseInTargetFrame(
      pose, pose_in_plan, *tf_, local_plan.header.frame_id, 0.2))
  {
    throw nav2_core::PlannerException("无法将机器人位姿转换到路径坐标系: " + local_plan.header.frame_id);
  }

  const size_t nearest_index = findNearestPathIndex(local_plan, pose_in_plan);
  const double velocity_based_lookahead =
    clamp(std::abs(velocity.linear.x) * lookahead_time_, min_lookahead_dist_, max_lookahead_dist_);
  const geometry_msgs::msg::PoseStamped lookahead_pose =
    selectLookaheadPose(local_plan, pose_in_plan, nearest_index, velocity_based_lookahead);

  geometry_msgs::msg::PoseStamped lookahead_in_base;
  if (!nav2_util::transformPoseInTargetFrame(
      lookahead_pose, lookahead_in_base, *tf_, costmap_ros_->getBaseFrameID(), 0.2))
  {
    throw nav2_core::PlannerException("无法将前视点转换到机器人坐标系");
  }

  const double dx = lookahead_in_base.pose.position.x;
  const double dy = lookahead_in_base.pose.position.y;
  const double heading_error = normalizeAngle(std::atan2(dy, std::max(dx, 1e-4)));
  const double lateral_error = dy;
  const auto & goal_pose = local_plan.poses.back().pose.position;
  const double goal_dist = std::hypot(
    goal_pose.x - pose_in_plan.pose.position.x, goal_pose.y - pose_in_plan.pose.position.y);

  geometry_msgs::msg::TwistStamped raw_cmd;
  raw_cmd.header.stamp = node_->now();
  raw_cmd.header.frame_id = costmap_ros_->getBaseFrameID();

  // 先检查正前方障碍，如果风险高则进入“墙边脱困”策略。
  const bool blocked = isForwardPathBlocked(pose, forward_check_dist_, forward_check_step_);
  if (blocked && goal_dist > 0.15) {
    const int turn_dir = chooseTurnDirection(pose);
    raw_cmd.twist.linear.x = (std::abs(heading_error) < 0.35) ? escape_linear_speed_ : 0.0;
    raw_cmd.twist.angular.z = static_cast<double>(turn_dir) * escape_angular_speed_;
  } else {
    // 当前视点在侧后方时，先原地调整朝向，避免硬推导致贴墙振荡。
    if (std::abs(heading_error) > rotate_in_place_min_angle_ && goal_dist > 0.30) {
      raw_cmd.twist.linear.x = 0.0;
      raw_cmd.twist.angular.z = clamp(heading_kp_ * heading_error, -max_angular_speed_, max_angular_speed_);
    } else {
      raw_cmd.twist.angular.z = clamp(
        heading_kp_ * heading_error + lateral_kp_ * lateral_error, -max_angular_speed_,
        max_angular_speed_);

      const double curvature_ratio = std::abs(raw_cmd.twist.angular.z) / max_angular_speed_;
      const double curvature_scale = clamp(1.0 - 0.75 * curvature_ratio, 0.20, 1.0);
      const double goal_scale = clamp(goal_dist / 0.80, 0.25, 1.0);
      raw_cmd.twist.linear.x = max_linear_speed_ * curvature_scale * goal_scale;

      if (goal_dist > 0.12) {
        raw_cmd.twist.linear.x = std::max(raw_cmd.twist.linear.x, min_linear_speed_);
      } else {
        raw_cmd.twist.linear.x = std::min(raw_cmd.twist.linear.x, 0.04);
      }
    }
  }

  return applyLimitsAndSmoothing(raw_cmd, raw_cmd.header.stamp);
}

void CustomController::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock(plan_mutex_);
  global_plan_ = path;
}

void CustomController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (percentage) {
    const double ratio = clamp(speed_limit / 100.0, 0.05, 1.0);
    max_linear_speed_ = std::max(min_linear_speed_, max_linear_speed_ * ratio);
  } else {
    max_linear_speed_ = clamp(speed_limit, min_linear_speed_, 1.0);
  }
}

size_t CustomController::findNearestPathIndex(
  const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & pose) const
{
  size_t nearest_index = 0;
  double min_dist = std::numeric_limits<double>::infinity();

  for (size_t i = 0; i < path.poses.size(); ++i) {
    const double dx = path.poses[i].pose.position.x - pose.pose.position.x;
    const double dy = path.poses[i].pose.position.y - pose.pose.position.y;
    const double d = std::hypot(dx, dy);
    if (d < min_dist) {
      min_dist = d;
      nearest_index = i;
    }
  }

  return nearest_index;
}

geometry_msgs::msg::PoseStamped CustomController::selectLookaheadPose(
  const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & pose, size_t start_index,
  double lookahead_dist) const
{
  for (size_t i = start_index; i < path.poses.size(); ++i) {
    const double dx = path.poses[i].pose.position.x - pose.pose.position.x;
    const double dy = path.poses[i].pose.position.y - pose.pose.position.y;
    if (std::hypot(dx, dy) >= lookahead_dist) {
      return path.poses[i];
    }
  }
  return path.poses.back();
}

bool CustomController::isForwardPathBlocked(
  const geometry_msgs::msg::PoseStamped & pose, double sample_dist, double sample_step) const
{
  if (!costmap_) {
    return false;
  }

  const double yaw = tf2::getYaw(pose.pose.orientation);
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);

  // 三条采样线：中线 + 左右偏置，用于快速估计“前方可通过空间”。
  const std::array<double, 3> lateral_offsets{{-0.08, 0.0, 0.08}};
  for (double dist = 0.08; dist <= sample_dist; dist += sample_step) {
    for (const double lat : lateral_offsets) {
      const double wx = pose.pose.position.x + c * dist - s * lat;
      const double wy = pose.pose.position.y + s * dist + c * lat;
      unsigned int mx = 0;
      unsigned int my = 0;
      if (!costmap_->worldToMap(wx, wy, mx, my)) {
        return true;
      }

      const unsigned char cost = costmap_->getCost(mx, my);
      if (cost == nav2_costmap_2d::NO_INFORMATION) {
        continue;
      }
      if (static_cast<int>(cost) >= obstacle_cost_threshold_) {
        return true;
      }
    }
  }

  return false;
}

int CustomController::chooseTurnDirection(const geometry_msgs::msg::PoseStamped & pose) const
{
  if (!costmap_) {
    return 1;
  }

  const double yaw = tf2::getYaw(pose.pose.orientation);
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);

  auto sample_cost = [&](double forward, double lateral) -> int {
      const double wx = pose.pose.position.x + c * forward - s * lateral;
      const double wy = pose.pose.position.y + s * forward + c * lateral;
      unsigned int mx = 0;
      unsigned int my = 0;
      if (!costmap_->worldToMap(wx, wy, mx, my)) {
        return 255;
      }
      return static_cast<int>(costmap_->getCost(mx, my));
    };

  const int left_cost = sample_cost(side_probe_dist_, 0.16);
  const int right_cost = sample_cost(side_probe_dist_, -0.16);
  return (left_cost <= right_cost) ? 1 : -1;
}

geometry_msgs::msg::TwistStamped CustomController::applyLimitsAndSmoothing(
  const geometry_msgs::msg::TwistStamped & raw_cmd, const rclcpp::Time & now)
{
  geometry_msgs::msg::TwistStamped limited_cmd = raw_cmd;

  double dt = (now - last_cmd_time_).seconds();
  if (dt <= 1e-3 || !std::isfinite(dt)) {
    dt = 0.1;
  }

  const double lin_step = max_linear_accel_ * dt;
  const double ang_step = max_angular_accel_ * dt;

  const double lin_target = clamp(
    raw_cmd.twist.linear.x, last_cmd_.twist.linear.x - lin_step, last_cmd_.twist.linear.x + lin_step);
  const double ang_target = clamp(
    raw_cmd.twist.angular.z, last_cmd_.twist.angular.z - ang_step,
    last_cmd_.twist.angular.z + ang_step);

  limited_cmd.twist.linear.x = clamp(lin_target, -max_linear_speed_, max_linear_speed_);
  limited_cmd.twist.angular.z = clamp(ang_target, -max_angular_speed_, max_angular_speed_);

  geometry_msgs::msg::TwistStamped smoothed_cmd = limited_cmd;
  smoothed_cmd.twist.linear.x =
    linear_velocity_smoothing_ * last_cmd_.twist.linear.x +
    (1.0 - linear_velocity_smoothing_) * limited_cmd.twist.linear.x;
  smoothed_cmd.twist.angular.z =
    angular_velocity_smoothing_ * last_cmd_.twist.angular.z +
    (1.0 - angular_velocity_smoothing_) * limited_cmd.twist.angular.z;

  // 不在旋转模式下时，避免线速度过小造成“看起来一直不动”的假停滞。
  if (std::abs(smoothed_cmd.twist.angular.z) < rotate_in_place_min_angle_ &&
    std::abs(smoothed_cmd.twist.linear.x) > 1e-4 &&
    std::abs(smoothed_cmd.twist.linear.x) < min_linear_speed_)
  {
    smoothed_cmd.twist.linear.x = std::copysign(min_linear_speed_, smoothed_cmd.twist.linear.x);
  }

  if (std::abs(smoothed_cmd.twist.linear.x) < 1e-4) {
    smoothed_cmd.twist.linear.x = 0.0;
  }
  if (std::abs(smoothed_cmd.twist.angular.z) < 1e-4) {
    smoothed_cmd.twist.angular.z = 0.0;
  }

  smoothed_cmd.header.stamp = now;
  smoothed_cmd.header.frame_id = costmap_ros_->getBaseFrameID();

  last_cmd_ = smoothed_cmd;
  last_cmd_time_ = now;
  return smoothed_cmd;
}

}  // namespace nav2_custom_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_custom_controller::CustomController, nav2_core::Controller)
