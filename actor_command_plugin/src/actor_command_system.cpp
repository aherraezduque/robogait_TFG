#include "actor_command_plugin/actor_command_system.hpp"

#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Actor.hh>
#include <gz/sim/Actor.hh>
#include <gz/sim/Util.hh>

#include <gz/sim/components/Name.hh>
#include <gz/sim/System.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Angle.hh>
#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace ignition_ros2_actor
{

    using namespace gz::sim;

    ////////////////////////////////////////////////////////////
    ActorCommandSystem::ActorCommandSystem() {}

    ActorCommandSystem::~ActorCommandSystem()
    {
        if (this->executor_)
        {
            this->executor_->cancel();
        }

        if (this->ros_spin_thread_.joinable())
        {
            this->ros_spin_thread_.join();
        }
    }


    ////////////////////////////////////////////////////////////
    void ActorCommandSystem::Configure(const Entity& entity,
        const std::shared_ptr<const sdf::Element>& sdf,
        EntityComponentManager& ecm,
        EventManager& /*eventMgr*/)
    {
        // Initialize Gazebo components, ROS interfaces and resources.
        this->actor_entity_ = entity;

        this->EnsureActorComponents(ecm);
        this->InitRosNode();
        this->LoadSdfParameters(sdf);
        this->CreateRosSubscriptions();
        this->CreateRosPublishers();
        this->StartRosExecutor();

    }

    ////////////////////////////////////////////////////////////
    void ActorCommandSystem::EnsureActorComponents(gz::sim::EntityComponentManager& ecm)
    {
        if (!ecm.EntityHasComponentType(this->actor_entity_, components::TrajectoryPose::typeId))
        {
            ecm.CreateComponent(this->actor_entity_, components::TrajectoryPose(ignition::math::Pose3d{}));
        }

    }

    ////////////////////////////////////////////////////////////
    void ActorCommandSystem::InitRosNode()
    {
        if (!rclcpp::ok())
        {
            rclcpp::init(0, nullptr);
        }

        this->node_ = std::make_shared<rclcpp::Node>("actor_command_system_node");

        RCLCPP_INFO(this->node_->get_logger(),
            "ActorCommandSystem Node Init");
    }

    ////////////////////////////////////////////////////////////
    void ActorCommandSystem::LoadSdfParameters(const std::shared_ptr<const sdf::Element>& sdf)
    {
        if (sdf->HasElement("follow_mode"))
            this->follow_mode_ = sdf->Get<std::string>("follow_mode");
        if (sdf->HasElement("default_rotation"))
            this->default_rotation_ = sdf->Get<double>("default_rotation");
        if (sdf->HasElement("initial_rotation"))
            this->initial_rotation_ = sdf->Get<double>("initial_rotation");
        this->rotation_pitch_ = this->initial_rotation_;

        // Topics for actor cmd
        if (sdf->HasElement("cmd_vel_topic"))
            this->vel_topic_ = sdf->Get<std::string>("cmd_vel_topic");
        if (sdf->HasElement("cmd_script_topic"))
            this->script_topic_ = sdf->Get<std::string>("cmd_script_topic");


        if (sdf->HasElement("robot_model_name"))
            this->robot_model_name_ = sdf->Get<std::string>("robot_model_name");
        if (sdf->HasElement("child_link_name"))
            this->child_link_name_ = sdf->Get<std::string>("child_link_name");

        // Topics for distance-pose publish
        if (sdf->HasElement("distance_topic"))
            this->distance_topic_ = sdf->Get<std::string>("distance_topic");
        if (sdf->HasElement("actor_pose_topic"))
            this->actor_pose_topic_ = sdf->Get<std::string>("actor_pose_topic");
        if (sdf->HasElement("robot_pose_topic"))
            this->robot_pose_topic_ = sdf->Get<std::string>("robot_pose_topic");

        if (sdf->HasElement("enable_distance_topic"))
            this->enable_distance_topic_ = sdf->Get<bool>("enable_distance_topic");
        if (sdf->HasElement("enable_actor_pose_topic"))
            this->enable_actor_pose_topic_ = sdf->Get<bool>("enable_actor_pose_topic");
        if (sdf->HasElement("enable_robot_pose_topic"))
            this->enable_robot_pose_topic_ = sdf->Get<bool>("enable_robot_pose_topic");

        // Actor pose offsets
        if (sdf->HasElement("actor_pose_offset_X"))
            this->actor_pose_offset_.X(sdf->Get<double>("actor_pose_offset_X"));
        if (sdf->HasElement("actor_pose_offset_Y"))
            this->actor_pose_offset_.Y(sdf->Get<double>("actor_pose_offset_Y"));
        if (sdf->HasElement("actor_pose_offset_Z"))
            this->actor_pose_offset_.Z(sdf->Get<double>("actor_pose_offset_Z"));
    }

    ////////////////////////////////////////////////////////////
    void ActorCommandSystem::CreateRosSubscriptions()
    {
        this->vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
            this->vel_topic_, 10,
            std::bind(&ActorCommandSystem::VelCallback, this, std::placeholders::_1));

        this->script_sub_ = node_->create_subscription<custom_msgs::msg::ActorTrajectoryPoint>(
            this->script_topic_, 10,
            std::bind(&ActorCommandSystem::ScriptCallback, this, std::placeholders::_1));
    }

    ////////////////////////////////////////////////////////////
    void ActorCommandSystem::CreateRosPublishers()
    {
        if (enable_distance_topic_) {
            this->distance_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
                this->distance_topic_, rclcpp::QoS(10));
        }

        if (enable_actor_pose_topic_) {
            this->actor_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
                this->actor_pose_topic_, rclcpp::QoS(10));
        }

        if (enable_robot_pose_topic_) {
            this->robot_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
                this->robot_pose_topic_, rclcpp::QoS(10));
        }
    }

    ////////////////////////////////////////////////////////////
    void ActorCommandSystem::StartRosExecutor()
    {
        // Spin ROS callbacks in a separate thread so Gazebo updates are not blocked
        this->executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        this->executor_->add_node(this->node_);

        this->ros_spin_thread_ = std::thread([this]() {
            this->executor_->spin();
            });
    }

    ////////////////////////////////////////////////////////////
    void ActorCommandSystem::VelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        this->current_linear_vel_ = msg->linear.x;
        this->current_angular_vel_ = msg->angular.z;
    }

    ////////////////////////////////////////////////////////////
    void ActorCommandSystem::ScriptCallback(const custom_msgs::msg::ActorTrajectoryPoint::SharedPtr msg)
    {

        first_time_ = false;

        TimedWaypoint twp;
        twp.x = msg->pose.position.x;
        twp.y = msg->pose.position.y;
        twp.z = msg->pose.position.z;

        twp.yaw = msg->pose.orientation.z;
        twp.t = msg->t;

        this->script_path_mutex_.lock();

        if (msg->clear || !this->defined_script_path_) {
            // Reset the script state when a new trajectory starts
            this->script_path_.clear();
            this->defined_script_path_ = true;

            this->has_active_segment_ = false;
            this->current_segment_ = ScriptSegment{};
            this->timed_idx_ = 0;

        }
        this->script_path_.push_back(twp);

        this->script_path_mutex_.unlock();

        RCLCPP_INFO(this->node_->get_logger(),
            "New waypoint: (%.3f, %.3f, %.3f), yaw = %.3f rad, t= %.3f, clear = %d ",
            twp.x, twp.y, twp.z, twp.yaw, twp.t, int(msg->clear));

    }

    ////////////////////////////////////////////////////////////
    void ActorCommandSystem::PreUpdate(const UpdateInfo& info, EntityComponentManager& ecm)
    {

        auto traj_ini = ecm.Component<gz::sim::components::TrajectoryPose>(this->actor_entity_);
        auto traj_ini_data = traj_ini->Data();

        gz::math::Pose3d currentPose, newPose;

        newPose.Pos().X() = currentPose.Pos().X() = traj_ini_data.Pos().X();
        newPose.Pos().Y() = currentPose.Pos().Y() = traj_ini_data.Pos().Y();
        newPose.Pos().Z() = currentPose.Pos().Z() = traj_ini_data.Pos().Z();

        double dt = std::chrono::duration<double>(info.dt).count();

        // Compute newPose according to follow_mode
        if (this->follow_mode_ == "velocity")
        {


            newPose.Pos().Z() += this->current_linear_vel_ * std::cos(this->rotation_pitch_) * dt;
            newPose.Pos().X() += this->current_linear_vel_ * std::sin(this->rotation_pitch_) * dt;

            this->rotation_pitch_ += this->current_angular_vel_ * dt;

            newPose.Rot() = gz::math::Quaterniond(0, this->rotation_pitch_, 0);

        }
        else if (this->follow_mode_ == "script")
        {
            if (first_time_) {

                newPose.Rot() = gz::math::Quaterniond(0, this->default_rotation_, 0);
            }
            else {
                newPose.Rot() = gz::math::Quaterniond(0, this->default_rotation_, 0);
                AdvanceScriptVelBased(dt, newPose);

                bool start_new_segment = false;
                TimedWaypoint A;
                TimedWaypoint B;

                // Copy waypoints under mutex because script_path_ is updated by the ROS executor thread
                this->script_path_mutex_.lock();

                // Start a new segment if the script path has not finished
                if (!has_active_segment_ && script_path_.size() > timed_idx_ + 1)
                {
                    A = this->script_path_[timed_idx_];
                    B = this->script_path_[timed_idx_ + 1];

                    timed_idx_++;
                    start_new_segment = true;
                }

                this->script_path_mutex_.unlock();

                if (start_new_segment)
                {
                    StartNewSegment(A, B, dt);
                }

            }

        }
        else
        {
            RCLCPP_WARN_ONCE(this->node_->get_logger(), "Unknown follow mode: '%s'. ", this->follow_mode_.c_str());
        }


        // Update TrajectoryPose
        auto trajPoseComp = ecm.Component<components::TrajectoryPose>(this->actor_entity_);
        if (trajPoseComp)
        {
            *trajPoseComp = components::TrajectoryPose(newPose);
        }
        else
        {
            ecm.CreateComponent(this->actor_entity_, components::TrajectoryPose(newPose));
        }
        ecm.SetChanged(this->actor_entity_, components::TrajectoryPose::typeId, ComponentState::OneTimeChange);

    }


    void ActorCommandSystem::PostUpdate(const UpdateInfo& info, const EntityComponentManager& ecm) {


        // Fill and check robot and child entities
        if (!this->CheckEntitiesFound(ecm)) {
            //Entities not found
            return;
        }

        // Get actor and robot model Poses
        auto actorTrajPose = ecm.Component<gz::sim::components::TrajectoryPose>(this->actor_entity_);
        if (!actorTrajPose) {
            RCLCPP_INFO(this->node_->get_logger(), "Actor TrajectoryPose component not found");
            return;
        }
        gz::math::Pose3d actorTrajData = actorTrajPose->Data();


        // Use configured child link pose or the robot model pose
        gz::math::Pose3d childWorldPose;

        if (this->child_link_name_ != "none") {
            childWorldPose = worldPose(this->child_entity_, ecm);
        }
        else {
            childWorldPose = worldPose(this->robot_entity_, ecm);
        }


        if (this->enable_actor_pose_topic_) {
            this->PublishActorPose(actorTrajData);
        }

        if (this->enable_robot_pose_topic_) {
            this->PublishRobotPose(childWorldPose);
        }

        if (this->enable_distance_topic_) {
            this->PublishDistance(actorTrajData, childWorldPose);
        }

    }

    void ActorCommandSystem::StartNewSegment(const TimedWaypoint& A, const TimedWaypoint& B, double dt) {

        double dx = B.y - A.y;
        double dz = B.x - A.x;
        double dist = std::hypot(dx, dz);
        double seg_dt = (B.t - A.t);

        if (seg_dt <= 0.0)
        {
            RCLCPP_WARN(this->node_->get_logger(), "Invalid script segment time: A.t = %.3f, B.t = %.3f", A.t, B.t);
            return;
        }

        if (dt <= 0.0)
        {
            RCLCPP_WARN(this->node_->get_logger(), "Invalid simulation dt: %.6f", dt);
            return;
        }

        // Create a constant-velocity segment between two timed waypoints
        this->rotation_pitch_ = A.yaw;
        current_segment_.A = A;
        current_segment_.B = B;

        // Compute linear vel and direction in this segment
        current_segment_.linear_vel = dist / seg_dt;
        current_segment_.yaw_motion = std::atan2(dx, dz);

        // Angular speed for smooth visual
        double dYaw = ShortestAngle(A.yaw, B.yaw);
        current_segment_.angular_vel_visual = dYaw / seg_dt;

        // nº of sim. steps to complete the segment = seg_dt / dt (0.001 Gazebo Ignition)
        current_segment_.steps_remaining = static_cast<int>(std::round(seg_dt / dt));

        has_active_segment_ = true;
    }

    void ActorCommandSystem::AdvanceScriptVelBased(double dt, gz::math::Pose3d& pose) {

        // Advance the actor along the active script segment

        if (current_segment_.steps_remaining > 0) {

            // Compute dz and dx 
            pose.Pos().Z() += std::cos(current_segment_.yaw_motion) * current_segment_.linear_vel * dt;
            pose.Pos().X() += std::sin(current_segment_.yaw_motion) * current_segment_.linear_vel * dt;

            // Compute new rotation_pitch
            this->rotation_pitch_ += current_segment_.angular_vel_visual * dt;
            pose.Rot() = gz::math::Quaterniond(0, this->rotation_pitch_, 0);

            // Segment velocity for optional animation/time-based logic
            current_linear_vel_ = current_segment_.linear_vel;

            current_segment_.steps_remaining--;

        }
        else {
            // Point B has been reached

            current_linear_vel_ = 0.0;
            current_angular_vel_ = 0.0;

            has_active_segment_ = false;
        }

    }

    bool ActorCommandSystem::CheckEntitiesFound(const gz::sim::EntityComponentManager& ecm) {

        if (this->robot_found_) {
            return true;
        }

        auto robot = ecm.EntityByComponents(components::Name(this->robot_model_name_));

        if (robot == gz::sim::kNullEntity) {

            RCLCPP_INFO(this->node_->get_logger(), "ROBOT MODEL (%s) NOT FOUND", this->robot_model_name_.c_str());
            return false;
        }
        // ROBOT MODEL NOT KNULL
        this->robot_entity_ = robot;
        this->robot_found_ = true;
        RCLCPP_INFO(this->node_->get_logger(), "ROBOT MODEL (%s) FOUND", this->robot_model_name_.c_str());

        // LINK POSE IS USED
        if (this->child_link_name_ != "none") {

            auto child = ecm.EntityByComponents(components::Name(this->child_link_name_));

            if (child == gz::sim::kNullEntity) {

                RCLCPP_INFO(this->node_->get_logger(), "CHILD LINK (%s) NOT FOUND", this->child_link_name_.c_str());
                this->robot_found_ = false;
                return false;
            }
            // CHILD ENTITY NOT KNULL
            this->child_entity_ = child;
            RCLCPP_INFO(this->node_->get_logger(), "CHILD LINK (%s) FOUND", this->child_link_name_.c_str());
        }

        return true;

    }

    gz::math::Pose3d ActorCommandSystem::GetActorWorldPose(const gz::math::Pose3d& actorPose)
    {
        gz::math::Pose3d actorWorldPose = actorPose;

        // Uses offsets to compute actorWorldPose, used by ROS topics
        actorWorldPose.Pos().X() = actorPose.Pos().Z() + this->actor_pose_offset_.X();
        actorWorldPose.Pos().Y() = actorPose.Pos().X() + this->actor_pose_offset_.Y();
        actorWorldPose.Pos().Z() = actorPose.Pos().Y() + this->actor_pose_offset_.Z();

        return actorWorldPose;
    }

    void ActorCommandSystem::PublishActorPose(const gz::math::Pose3d& actorTrajData) {

        if (!this->actor_pose_pub_) {
            return;
        }
        geometry_msgs::msg::PoseStamped p;
        p.header.stamp = this->node_->get_clock()->now();
        p.header.frame_id = "world";

        // Uses offsets to compute actorWorldPose, used by ROS topics
        gz::math::Pose3d actorWorldPose = GetActorWorldPose(actorTrajData);

        p.pose.position.x = actorWorldPose.X();
        p.pose.position.y = actorWorldPose.Y();
        p.pose.position.z = actorWorldPose.Z();

        p.pose.orientation.x = actorWorldPose.Rot().X();
        p.pose.orientation.y = actorWorldPose.Rot().Y();
        p.pose.orientation.z = actorWorldPose.Rot().Z();
        p.pose.orientation.w = actorWorldPose.Rot().W();

        this->actor_pose_pub_->publish(p);
    }

    void ActorCommandSystem::PublishRobotPose(const gz::math::Pose3d& childWorldPose) {

        if (!this->robot_pose_pub_) {
            return;
        }
        geometry_msgs::msg::PoseStamped p;
        p.header.stamp = this->node_->get_clock()->now();
        p.header.frame_id = "world";

        p.pose.position.x = childWorldPose.Pos().X();
        p.pose.position.y = childWorldPose.Pos().Y();
        p.pose.position.z = childWorldPose.Pos().Z();

        p.pose.orientation.x = childWorldPose.Rot().X();
        p.pose.orientation.y = childWorldPose.Rot().Y();
        p.pose.orientation.z = childWorldPose.Rot().Z();
        p.pose.orientation.w = childWorldPose.Rot().W();

        this->robot_pose_pub_->publish(p);
    }

    void ActorCommandSystem::PublishDistance(gz::math::Pose3d& actorTrajData, const gz::math::Pose3d& childWorldPose) {

        if (!this->distance_pub_) {
            return;
        }


        gz::math::Pose3d actorWorldPose = GetActorWorldPose(actorTrajData);

        double distance = actorWorldPose.Pos().Distance(childWorldPose.Pos());

        std_msgs::msg::Float64 msg;
        msg.data = distance;
        this->distance_pub_->publish(msg);

    }

    double ActorCommandSystem::ShortestAngle(double from, double to) {

        double diff = to - from;

        while (diff < -M_PI) {
            diff += 2.0 * M_PI;
        }

        while (diff > M_PI) {
            diff -= 2.0 * M_PI;
        }

        return diff;
    }

}  // namespace ignition_ros2_actor


IGNITION_ADD_PLUGIN(ignition_ros2_actor::ActorCommandSystem,
    ignition::gazebo::System,
    ignition::gazebo::ISystemConfigure,
    ignition::gazebo::ISystemPreUpdate,
    ignition::gazebo::ISystemPostUpdate)

    IGNITION_ADD_PLUGIN_ALIAS(ignition_ros2_actor::ActorCommandSystem,
        "ignition::gazebo::systems::ActorCommandSystem")
