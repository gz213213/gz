#ifndef NAV2_CUSTOM_PLANNER__NAV2_CUSTOM_PLANNER_HPP_
#define NAV2_CUSTOM_PLANNER__NAV2_CUSTOM_PLANNER_HPP_



#include <memory>
#include <string>
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav_msgs/msg/path.hpp"

namespace nav2_custom_planner {
// 自定义导航规划器类
class CustomPlanner : public nav2_core::GlobalPlanner {
public:
  CustomPlanner() = default;
  ~CustomPlanner() = default;//默认析构函数
  // 插件配置方法
  void configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,//父节点指针和插件名称
      std::shared_ptr<tf2_ros::Buffer> tf,//坐标变换缓存指针    
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;//代价地图指针
  // 插件清理方法
  void cleanup() override;
  // 插件激活方法
  void activate() override;
  // 插件停用方法
  void deactivate() override;
  // 为给定的起始和目标位姿创建路径的方法
  nav_msgs::msg::Path
  createPlan(const geometry_msgs::msg::PoseStamped &start,
             const geometry_msgs::msg::PoseStamped &goal) override;

private:
  // 坐标变换缓存指针，可用于查询坐标关系
  std::shared_ptr<tf2_ros::Buffer> tf_;
  // 节点指针
  nav2_util::LifecycleNode::SharedPtr node_;
  // 全局代价地图
  nav2_costmap_2d::Costmap2D *costmap_;
  // 全局代价地图的坐标系
  std::string global_frame_, name_;
  // 插值分辨率
  double interpolation_resolution_;
  // 是否使用曼哈顿距离作为启发式函数
  bool use_manhattan_distance_;
  // 是否启用路径平滑
  bool enable_path_smoothing_;
  // 最大搜索迭代次数
  int max_search_iterations_;
  // 是否允许走未知区域（安全优先默认 false）
  bool allow_unknown_;
  // 是否允许对角穿角（防穿墙默认 false）
  bool allow_corner_cutting_;
  // 代价值阈值：>= 阈值视为不可通行
  int lethal_cost_threshold_;
  // 兼容旧参数（已弃用）：障碍代价惩罚权重。
  double obstacle_cost_weight_;
  // 路径平滑时的安全代价上限：>= 该值则禁止直连，避免贴墙“抹直”
  int smoothing_max_cost_;

  // 自适应总代价参数
  double eps_;
  double lambda_c_min_;
  double lambda_c_max_;
  double lambda_m_min_;
  double lambda_m_max_;
  double lambda_t_min_;
  double lambda_t_max_;
  double sigma_c_;
  double sigma_m_;
  double sigma_t_;
  double local_width_scale_;
};

} // namespace nav2_custom_planner






#endif // NAV2_CUSTOM_PLANNER__NAV2_CUSTOM_PLANNER_HPP_
