/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Author(s): Shrijit Singh <shrijitsingh99@gmail.com>
 *  Contributor: Pham Cong Trang <phamcongtranghd@gmail.com>
 *  Contributor: Mitchell Sayer <mitchell4408@gmail.com>
 */

#include <algorithm>
#include <string>
#include <memory>

 //#include "nav2_core/controller_exceptions.hpp"
 //#include "nav2_core/planner_exceptions.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_pure_pursuit_controller_custom/pure_pursuit_controller.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "pluginlib/class_list_macros.hpp" // mine
//#include "navigation_pkg/msg/user.hpp"

#include <fstream>
#include <iomanip>
#include <sstream>

using std::hypot;
using std::min;
using std::max;
using std::abs;
using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;

namespace nav2_pure_pursuit_controller
{

  /**
   * Find element in iterator with the minimum calculated value
   */
  template<typename Iter, typename Getter>
  Iter min_by(Iter begin, Iter end, Getter getCompareVal)
  {
    if (begin == end) {
      return end;
    }
    auto lowest = getCompareVal(*begin);
    Iter lowest_it = begin;
    for (Iter it = ++begin; it != end; ++it) {
      auto comp = getCompareVal(*it);
      if (comp < lowest) {
        lowest = comp;
        lowest_it = it;
      }
    }
    return lowest_it;
  }

  void PurePursuitController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
    std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {
    node_ = parent;

    auto node = node_.lock();

    costmap_ros_ = costmap_ros;
    tf_ = tf;
    plugin_name_ = name;
    logger_ = node->get_logger();
    clock_ = node->get_clock();

    declare_parameter_if_not_declared(
      node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(
        0.2));
    declare_parameter_if_not_declared(
      node, plugin_name_ + ".lookahead_dist",
      rclcpp::ParameterValue(0.4));
    declare_parameter_if_not_declared(
      node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(
        1.0));
    declare_parameter_if_not_declared(
      node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(
        0.1));

    node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
    node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
    node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
    double transform_tolerance;
    node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
    transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);

    global_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);

    user_sub_ = node->create_subscription<std_msgs::msg::Float64>("user_coordinates", 10, std::bind(&PurePursuitController::userCoordinatesCallback, this, std::placeholders::_1));
  }

  void PurePursuitController::cleanup()
  {
    RCLCPP_INFO(
      logger_,
      "Cleaning up controller: %s of type pure_pursuit_controller::PurePursuitController",
      plugin_name_.c_str());
    global_pub_.reset();
  }

  void PurePursuitController::activate()
  {
    RCLCPP_INFO(
      logger_,
      "Activating controller: %s of type pure_pursuit_controller::PurePursuitController\"  %s",
      plugin_name_.c_str(), plugin_name_.c_str());
    global_pub_->on_activate();
    RCLCPP_INFO(logger_, "PurePursuitController activated");
  }

  void PurePursuitController::deactivate()
  {
    RCLCPP_INFO(
      logger_,
      "Dectivating controller: %s of type pure_pursuit_controller::PurePursuitController\"  %s",
      plugin_name_.c_str(), plugin_name_.c_str());
    global_pub_->on_deactivate();
  }

  void PurePursuitController::setSpeedLimit(const double& speed_limit, const bool& percentage)
  {
    (void)speed_limit;
    (void)percentage;
  }

  geometry_msgs::msg::TwistStamped PurePursuitController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped& pose,
    const geometry_msgs::msg::Twist& velocity,
    nav2_core::GoalChecker* goal_checker)
  {
    double timestamp = clock_->now().seconds();

    (void)velocity;
    (void)goal_checker;

    auto transformed_plan = transformGlobalPlan(pose);

    auto goal_pose_it = std::find_if(
      transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto& ps) {
        return hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist_;
      });

    if (goal_pose_it == transformed_plan.poses.end()) {
      goal_pose_it = std::prev(transformed_plan.poses.end());
    }
    auto goal_pose = goal_pose_it->pose;

    double linear_vel = 0.0;
    double angular_vel = 0.0;

    // =========================
    // Parámetros de distancia objetivo (zona muerta: 3.0 - 3.2)
    // =========================
    const double target_distance_min = 3.0;
    const double target_distance_max = 3.2;
    // const double target_distance = (target_distance_min + target_distance_max) / 2.0; // 3.1 punto medio

    // Si no debe moverse hacia la persona, detener inmediatamente
    if (!move_to_user_) {
      RCLCPP_INFO(logger_, "StopRobot(): distancia persona %.2f m", distance_person_);
      geometry_msgs::msg::TwistStamped stop_cmd;
      stop_cmd.header.frame_id = pose.header.frame_id;
      stop_cmd.header.stamp = clock_->now();
      stop_cmd.twist.linear.x = 0.0;
      stop_cmd.twist.angular.z = 0.0;
      return stop_cmd;
    }

    // =========================
    // Árbol de decisiones
    // =========================
    if (distance_person_ > 3.2 && distance_person_ <= 4.5) {
      // Retroceder
      linear_vel = -0.2;//-desired_linear_vel_;
      angular_vel = 0.0; // retroceso recto
      RCLCPP_WARN(logger_, "RETROCEDER. Distancia %.2f m", distance_person_);
      savePlansToCSV(timestamp, global_plan_, transformed_plan, goal_pose, linear_vel, angular_vel, 0.0, "RETROCEDER", distance_person_);

    }
    else if (distance_person_ >= target_distance_min && distance_person_ <= target_distance_max) {
      // Zona muerta
      linear_vel = 0.0;
      angular_vel = 0.0;
      RCLCPP_WARN(logger_, "DETENER (zona muerta). Distancia %.2f m", distance_person_);
      savePlansToCSV(timestamp, global_plan_, transformed_plan, goal_pose, linear_vel, angular_vel, 0.0, "DETENER_ZONA_MUERTA", distance_person_);

    }
    else if (distance_person_ > 1.5 && distance_person_ < target_distance_min) {
      // =========================
      // Avanzar  aplicar Pure Pursuit con normalización de velocidad
      // =========================
      double angle_to_goal = std::atan2(goal_pose.position.y, goal_pose.position.x);
      double curvature = 0.0;
      double denominator = goal_pose.position.x * goal_pose.position.x +
        goal_pose.position.y * goal_pose.position.y;
      if (denominator > 0.0) {
        curvature = 2.0 * goal_pose.position.y / denominator;
      }

      const double angle_threshold_30 = M_PI / 6.0;  // 30°
      const double angle_threshold_15 = M_PI / 12.0; // 15°

      // =========================
      // Escalado lineal de velocidad según distancia
      // =========================
      double min_dist = 1.5;   // cuando está cerca → velocidad máxima
      double max_dist = 3.0;   // cuando empieza a avanzar → velocidad cero
      double v = (max_dist - distance_person_) / (max_dist - min_dist);
      v = std::clamp(v, 0.0, 1.0);

      // interpolación suave con saturación
      double cmd_linear = v * desired_linear_vel_;

      // =========================
      // Decisiones angulares con PID
      // =========================
      static double integral_error = 0.0;
      static double prev_error = 0.0;
      double dt = 0.05; // suponiendo 20 Hz, ajústalo a tu frecuencia real

      double error = angle_to_goal;
      integral_error += error * dt;
      double derivative = (error - prev_error) / dt;

      // Ganancias PID angular (ajustables)
      double angular_kp = 1.6;
      double angular_ki = 0.05;
      double angular_kd = 0.01;

      double cmd_angular_pid = angular_kp * error +
        angular_ki * integral_error +
        angular_kd * derivative;

      prev_error = error;

      if (std::abs(angle_to_goal) > angle_threshold_30) {
        linear_vel = 0.0;
        //angular_vel = std::clamp(angular_kp * angle_to_goal, -max_angular_vel_, max_angular_vel_);
        angular_vel = std::clamp(cmd_angular_pid, -max_angular_vel_, max_angular_vel_);
        RCLCPP_WARN(logger_, "SOLO ROTAR. Distancia %.2f m | angle %.3f rad", distance_person_, angle_to_goal);
        savePlansToCSV(timestamp, global_plan_, transformed_plan, goal_pose, linear_vel, angular_vel, curvature, "SOLO_ROTAR", distance_person_);

      }
      else if (std::abs(angle_to_goal) < angle_threshold_15) {
        linear_vel = cmd_linear;
        angular_vel = 0.0;
        RCLCPP_WARN(logger_, "AVANZAR RECTO (normalizado). Distancia %.2f m | v=%.3f", distance_person_, linear_vel);
        savePlansToCSV(timestamp, global_plan_, transformed_plan, goal_pose, linear_vel, angular_vel, curvature, "AVANZAR_RECTO", distance_person_);

      }
      else {
        linear_vel = cmd_linear;
        //angular_vel = std::clamp(linear_vel * curvature, -max_angular_vel_, max_angular_vel_);
        angular_vel = std::clamp(cmd_angular_pid, -max_angular_vel_, max_angular_vel_);
        RCLCPP_WARN(logger_, "AVANZAR CON CURVA (normalizado). Distancia %.2f m | v=%.3f | curv=%.3f", distance_person_, linear_vel, curvature);
        savePlansToCSV(timestamp, global_plan_, transformed_plan, goal_pose, linear_vel, angular_vel, curvature, "AVANZAR_CURVA", distance_person_);
      }
    }

    else {
      // Detener por defecto (demasiado lejos)
      linear_vel = 0.0;
      angular_vel = 0.0;
      RCLCPP_WARN(logger_, "DETENER. Distancia %.2f m", distance_person_);
      savePlansToCSV(timestamp, global_plan_, transformed_plan, goal_pose, linear_vel, angular_vel, 0.0, "DETENER", distance_person_);
    }

    // =========================
    // Construcción comando
    // =========================
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.frame_id = pose.header.frame_id;
    cmd_vel.header.stamp = clock_->now();
    cmd_vel.twist.linear.x = linear_vel;
    cmd_vel.twist.angular.z = std::clamp(angular_vel, -max_angular_vel_, max_angular_vel_);

    return cmd_vel;
  }

  void PurePursuitController::setPlan(const nav_msgs::msg::Path& path)
  {
    global_pub_->publish(path);
    global_plan_ = path;
  }

  nav_msgs::msg::Path
    PurePursuitController::transformGlobalPlan(
      const geometry_msgs::msg::PoseStamped& pose)
  {
    if (global_plan_.poses.empty()) {
      throw std::runtime_error("Received plan with zero length");
    }
    geometry_msgs::msg::PoseStamped robot_pose;
    if (!transformPose(
      tf_, global_plan_.header.frame_id, pose,
      robot_pose, transform_tolerance_))
    {
      throw std::runtime_error("Unable to transform robot pose into global plan's frame");
    }

    nav2_costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
    double dist_threshold = std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()) *
      costmap->getResolution() / 2.0;

    auto transformation_begin =
      min_by(
        global_plan_.poses.begin(), global_plan_.poses.end(),
        [&robot_pose](const geometry_msgs::msg::PoseStamped& ps) {
          return euclidean_distance(robot_pose, ps);
        });

    auto transformation_end = std::find_if(
      transformation_begin, end(global_plan_.poses),
      [&](const auto& global_plan_pose) {
        return euclidean_distance(robot_pose, global_plan_pose) > dist_threshold;
      });

    auto transformGlobalPoseToLocal = [&](const auto& global_plan_pose) {
      // We took a copy of the pose, let's lookup the transform at the current time
      geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
      stamped_pose.header.frame_id = global_plan_.header.frame_id;
      stamped_pose.header.stamp = pose.header.stamp;
      stamped_pose.pose = global_plan_pose.pose;
      transformPose(
        tf_, costmap_ros_->getBaseFrameID(),
        stamped_pose, transformed_pose, transform_tolerance_);
      return transformed_pose;
      };

    nav_msgs::msg::Path transformed_plan;
    std::transform(
      transformation_begin, transformation_end,
      std::back_inserter(transformed_plan.poses),
      transformGlobalPoseToLocal);
    transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
    transformed_plan.header.stamp = pose.header.stamp;

    global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
    global_pub_->publish(transformed_plan);

    if (transformed_plan.poses.empty()) {
      //throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
      //throw Nav2Exception("Resulting plan has 0 poses in it.");
      throw std::runtime_error("Resulting plan has 0 poses in it.");
    }

    return transformed_plan;
  }

  bool PurePursuitController::transformPose(
    const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::string frame,
    const geometry_msgs::msg::PoseStamped& in_pose,
    geometry_msgs::msg::PoseStamped& out_pose,
    const rclcpp::Duration& transform_tolerance
  ) const
  {
    // Implementation taken as is fron nav_2d_utils in nav2_dwb_controller

    if (in_pose.header.frame_id == frame) {
      out_pose = in_pose;
      return true;
    }

    try {
      tf->transform(in_pose, out_pose, frame);
      return true;
    }
    catch (tf2::ExtrapolationException& ex) {
      auto transform = tf->lookupTransform(
        frame,
        in_pose.header.frame_id,
        tf2::TimePointZero
      );
      if (
        (rclcpp::Time(in_pose.header.stamp) - rclcpp::Time(transform.header.stamp)) >
        transform_tolerance)
      {
        RCLCPP_ERROR(
          rclcpp::get_logger("tf_help"),
          "Transform data too old when converting from %s to %s",
          in_pose.header.frame_id.c_str(),
          frame.c_str()
        );
        RCLCPP_ERROR(
          rclcpp::get_logger("tf_help"),
          "Data time: %ds %uns, Transform time: %ds %uns",
          in_pose.header.stamp.sec,
          in_pose.header.stamp.nanosec,
          transform.header.stamp.sec,
          transform.header.stamp.nanosec
        );
        return false;
      }
      else {
        tf2::doTransform(in_pose, out_pose, transform);
        return true;
      }
    }
    catch (tf2::TransformException& ex) {
      RCLCPP_ERROR(
        rclcpp::get_logger("tf_help"),
        "Exception in transformPose: %s",
        ex.what()
      );
      return false;
    }
    return false;
  }

  /*
  * M E T O D O S
  */

  rclcpp::Time last_time_;
  double last_distance_ = 0.0; //-1.0; funciona pero a veces rota sin conocer la causa
  double user_speed_ = 0.0;

  void PurePursuitController::userCoordinatesCallback(const std_msgs::msg::Float64::SharedPtr msg)
    //void PurePursuitController::userCoordinatesCallback(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    double current_distance = msg->data;
    rclcpp::Time current_time = clock_->now();

    if (last_distance_ > 0.0) {
      double delta_time = (current_time - last_time_).seconds(); // en segundos
      double delta_distance = current_distance - last_distance_; // positivo = alejándose
      user_speed_ = delta_distance / delta_time;
    }

    // Actualizar últimos valores
    last_distance_ = current_distance;
    last_time_ = current_time;
    distance_person_ = msg->data;

    if (distance_person_ <= 0.0) {
      move_to_user_ = false;
      RCLCPP_INFO(
        logger_,
        "- No se detecta persona. move_to_user_: %s | distancia: %.2f m",
        move_to_user_ ? "true" : "false",
        //move_to_user_ = false, 
        distance_person_
      );
    }

    // Condición para detener al robot, ej. si persona está muy cerca o muy lejos
    else if (distance_person_ < 1.5 || distance_person_ > 4.5) {
      //stop_robot_ = true;

      move_to_user_ = false;
      RCLCPP_INFO(
        logger_,
        "- Persona fuera del rango permitido. move_to_user_: %s | distancia: %.2f m",
        move_to_user_ ? "true" : "false",
        distance_person_
      );
    }
    else {
      // significa que ve persona dentro del rango permitido
      move_to_user_ = true;
      RCLCPP_INFO(
        logger_,
        "+ Persona DENTRO del rango permitido. move_to_user_: %s | distancia: %.2f m",
        move_to_user_ ? "true" : "false",
        distance_person_
      );
    }
  }


  void PurePursuitController::StopRobot()
  {
    RCLCPP_WARN(logger_, "Ejecutando StopRobot: deteniendo robot");
    // linear_vel = 0.0; // evitamos que avance, retroceda o rote.
    geometry_msgs::msg::TwistStamped stop_cmd;
    stop_cmd.header.frame_id = costmap_ros_->getBaseFrameID(); //pose.header.frame_id;
    stop_cmd.header.stamp = clock_->now();
    stop_cmd.twist.linear.x = 0.0;
    stop_cmd.twist.angular.z = 0.0;
  }

  bool PurePursuitController::shouldMoveToUser()
  {
    if (distance_person_ <= 0.0) {
      return false;  // No detecta a la persona
    }

    if (distance_person_ >= 1.5 && distance_person_ <= 4.5) {
      return true;  // Persona en rango
    }

    // Persona fuera del rango pero detectada => queremos movernos hasta estar a 3 metros
    return true;
  }

  void PurePursuitController::savePlansToCSV(
    double timestamp,
    const nav_msgs::msg::Path& global_plan,
    const nav_msgs::msg::Path& transformed_plan,
    const geometry_msgs::msg::Pose& lookahead_point,
    double linear_vel,
    double angular_vel,
    double curvature,
    const std::string& action,
    double distance_person)
  {
    static bool header_written = false;

    static bool first_call = true;
    std::ios_base::openmode mode = std::ios::app;
    if (first_call) {
      mode = std::ios::trunc; // borra archivo viejo
      first_call = false;
    }

    std::ofstream file("/home/robogait/rover_workspace/src/nav2_pure_pursuit_controller/pure_pursuit_log.csv", mode);

    if (!file.is_open()) {
      RCLCPP_ERROR(logger_, "No se pudo abrir el archivo CSV para escritura.");
      return;
    }

    if (!header_written) {
      file << "timestamp,global_x,global_y,local_x,local_y,lookahead_x,lookahead_y,"
        << "linear_vel,angular_vel,curvature,action,distance_person\n";
      header_written = true;
    }

    size_t max_size = std::max(global_plan.poses.size(), transformed_plan.poses.size());

    for (size_t i = 0; i < max_size; ++i) {
      std::ostringstream line;

      // Global plan
      if (i < global_plan.poses.size()) {
        const auto& p = global_plan.poses[i].pose.position;
        line << p.x << "," << p.y << ",";
      }
      else {
        line << ",,";
      }

      // Local plan
      if (i < transformed_plan.poses.size()) {
        const auto& p = transformed_plan.poses[i].pose.position;
        line << p.x << "," << p.y << ",";
      }
      else {
        line << ",,";
      }

      // Lookahead solo una vez (fila 0)
      if (i == 0) {
        const auto& p = lookahead_point.position;
        line << p.x << "," << p.y << ",";
        // → datos de control (solo en primera fila)
        line << timestamp << ","
          << linear_vel << ","
          << angular_vel << ","
          << curvature << ","
          << action << ","
          << distance_person;
      }
      else {
        line << ",,,,,";
      }

      file << line.str() << "\n";
    }

    file.close();
  }

}  // namespace nav2_pure_pursuit_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(nav2_pure_pursuit_controller::PurePursuitController, nav2_core::Controller)
