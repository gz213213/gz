#ifndef NAV2_CUSTOM_CONTROLLER__CUSTOM_CONTROLLER_HPP_
#define NAV2_CUSTOM_CONTROLLER__CUSTOM_CONTROLLER_HPP_

#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_custom_controller
{

class CustomController : public nav2_core::Controller
{
public:
  CustomController() = default;
  ~CustomController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  // 在全局路径中找到与当前位姿最近的索引，用于路径剪枝。
  size_t findNearestPathIndex(
    const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & pose) const;

  // 从最近点向前搜索前视点，避免跟踪“身后路径”导致抖动。
  geometry_msgs::msg::PoseStamped selectLookaheadPose(
    const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & pose, size_t start_index,
    double lookahead_dist) const;

  // 检查机器人正前方扇区是否可通行，用于触发“脱困模式”。
  bool isForwardPathBlocked(
    const geometry_msgs::msg::PoseStamped & pose, double sample_dist, double sample_step) const;

  // 比较左右侧代价值，决定卡墙时优先向哪一侧旋转。
  int chooseTurnDirection(const geometry_msgs::msg::PoseStamped & pose) const;

  // 统一限幅和加速度约束，保证底层执行稳定。
  geometry_msgs::msg::TwistStamped applyLimitsAndSmoothing(
    const geometry_msgs::msg::TwistStamped & raw_cmd, const rclcpp::Time & now);

  static double normalizeAngle(double angle);
  static double clamp(double value, double low, double high);

  std::string plugin_name_;
  nav2_util::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};

  mutable std::mutex plan_mutex_;
  nav_msgs::msg::Path global_plan_;

  // 速度与加速度上限
  double max_linear_speed_{0.15};
  double min_linear_speed_{0.04};
  double max_angular_speed_{1.0};
  double max_linear_accel_{0.45};
  double max_angular_accel_{1.5};

  // 路径跟踪相关参数
  double min_lookahead_dist_{0.20};
  double max_lookahead_dist_{0.60};
  double lookahead_time_{1.2};
  double heading_kp_{1.8};
  double lateral_kp_{1.2};
  double rotate_in_place_min_angle_{0.55};

  // 碰撞/脱困相关参数
  double forward_check_dist_{0.40};
  double forward_check_step_{0.05};
  double side_probe_dist_{0.20};
  int obstacle_cost_threshold_{200};
  double escape_angular_speed_{0.45};
  double escape_linear_speed_{0.03};

  // 速度平滑参数（指数平滑，兼顾响应与稳定）
  double linear_velocity_smoothing_{0.70};
  double angular_velocity_smoothing_{0.65};

  geometry_msgs::msg::TwistStamped last_cmd_;
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace nav2_custom_controller

#endif  // NAV2_CUSTOM_CONTROLLER__CUSTOM_CONTROLLER_HPP_
