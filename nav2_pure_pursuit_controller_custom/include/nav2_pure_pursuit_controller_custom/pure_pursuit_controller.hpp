/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Author(s): Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#ifndef NAV2_PURE_PURSUIT_CONTROLLER__PURE_PURSUIT_CONTROLLER_HPP_
#define NAV2_PURE_PURSUIT_CONTROLLER__PURE_PURSUIT_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
 //#include "navigation_pkg/msg/user.hpp"
#include "std_msgs/msg/float64.hpp"

namespace nav2_pure_pursuit_controller
{

  class PurePursuitController : public nav2_core::Controller
  {
  public:
    PurePursuitController() = default;
    ~PurePursuitController() override = default;

    void configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
      std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
      const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;


    void cleanup() override;
    void activate() override;
    void deactivate() override;
    void setSpeedLimit(const double& speed_limit, const bool& percentage) override;

    geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped& pose,
      const geometry_msgs::msg::Twist& velocity,
      nav2_core::GoalChecker* goal_checker) override;

    void setPlan(const nav_msgs::msg::Path& path) override;

    // Callback del topic "user_coordinates"
    void userCoordinatesCallback(const std_msgs::msg::Float64::SharedPtr msg);

    // Guardar datos de trayectorias en CSV (para análisis o debugging)
    void savePlansToCSV(
      double timestamp,
      const nav_msgs::msg::Path& global_plan,
      const nav_msgs::msg::Path& transformed_plan,
      const geometry_msgs::msg::Pose& goal_pose,   // <-- CAMBIAR A Pose
      double linear_vel,
      double angular_vel,
      double curvature,
      const std::string& action,
      double distance_person);

    // Parar el robot completamente
    void StopRobot();

    // Lógica para decidir si moverse hacia el usuario
    bool shouldMoveToUser();
  protected:
    nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped& pose);

    bool transformPose(
      const std::shared_ptr<tf2_ros::Buffer> tf,
      const std::string frame,
      const geometry_msgs::msg::PoseStamped& in_pose,
      geometry_msgs::msg::PoseStamped& out_pose,
      const rclcpp::Duration& transform_tolerance
    ) const;

    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::string plugin_name_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    rclcpp::Logger logger_{ rclcpp::get_logger("PurePursuitController") };
    rclcpp::Clock::SharedPtr clock_;

    double desired_linear_vel_;
    double lookahead_dist_;
    double max_angular_vel_;
    rclcpp::Duration transform_tolerance_{ 0, 0 };

    nav_msgs::msg::Path global_plan_;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_pub_;

  private:
    // Suscripción al topic del usuario
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr user_sub_;

    // Última distancia detectada a la persona
    double distance_person_ = 0.0;

    // Flag para saber si el robot debe moverse hacia el usuario
    bool move_to_user_ = false;
  };

}  // namespace nav2_pure_pursuit_controller

#endif  // NAV2_PURE_PURSUIT_CONTROLLER__PURE_PURSUIT_CONTROLLER_HPP_