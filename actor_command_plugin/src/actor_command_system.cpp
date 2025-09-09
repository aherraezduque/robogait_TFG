#include "actor_command_plugin/actor_command_system.hpp"

#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Actor.hh>
#include <gz/sim/Actor.hh>
/* #include <ignition/gazebo/Actor.hh> */

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

        this->actor_entity_ = entity;

        if (!ecm.EntityHasComponentType(this->actor_entity_, components::TrajectoryPose::typeId))
        {
            ecm.CreateComponent(this->actor_entity_, components::TrajectoryPose(ignition::math::Pose3d{}));
        }

        // Crear componente AnimationTime si no existe
        if (!ecm.EntityHasComponentType(this->actor_entity_, components::AnimationTime::typeId))
        {
            ecm.CreateComponent(this->actor_entity_, components::AnimationTime(std::chrono::steady_clock::duration::zero()));
        }


        // Necesario en caso de que no se haya lanzado ROS2 antes 
        if (!rclcpp::ok())
        {
            rclcpp::init(0, nullptr);
        }

        this->node_ = std::make_shared<rclcpp::Node>("actor_command_system_node");

        if (sdf->HasElement("follow_mode"))
            this->follow_mode_ = sdf->Get<std::string>("follow_mode");
        if (sdf->HasElement("linear_velocity"))
            this->lin_velocity_ = sdf->Get<double>("linear_velocity");
        if (sdf->HasElement("angular_velocity"))
            this->ang_velocity_ = sdf->Get<double>("angular_velocity");
        if (sdf->HasElement("linear_tolerance"))
            this->lin_tolerance_ = sdf->Get<double>("linear_tolerance");
        if (sdf->HasElement("angular_tolerance"))
            this->ang_tolerance_ = sdf->Get<double>("angular_tolerance");
        if (sdf->HasElement("animation_factor"))
            this->animation_factor_ = sdf->Get<double>("animation_factor");
        if (sdf->HasElement("default_rotation"))
            this->default_rotation_ = sdf->Get<double>("default_rotation");

        // Topics for actor cmd
        if (sdf->HasElement("cmd_vel_topic"))
            this->vel_topic_ = sdf->Get<std::string>("cmd_vel_topic");
        if (sdf->HasElement("cmd_path_topic"))
            this->path_topic_ = sdf->Get<std::string>("cmd_path_topic");
        if (sdf->HasElement("cmd_animation_topic"))
            this->animation_topic_ = sdf->Get<std::string>("cmd_animation_topic");

        // Topics for distance-pose publish
        if (sdf->HasElement("robot_model_name"))
            this->robot_model_name_ = sdf->Get<std::string>("robot_model_name");
        if (sdf->HasElement("child_link_name"))
            this->child_link_name_ = sdf->Get<std::string>("child_link_name");

        if (sdf->HasElement("distance_topic"))
            this->robot_model_name_ = sdf->Get<std::string>("distance_topic");
        if (sdf->HasElement("actor_pose_topic"))
            this->robot_model_name_ = sdf->Get<std::string>("actor_pose_topic");
        if (sdf->HasElement("robot_pose_topic"))
            this->robot_model_name_ = sdf->Get<std::string>("robot_pose_topic");

        // Actor pose offsets
        if (sdf->HasElement("actor_pose_offset_X"))
            this->actor_pose_offset_X_ = sdf->Get<double>("actor_pose_offset_X");
        if (sdf->HasElement("actor_pose_offset_Y"))
            this->actor_pose_offset_Y_ = sdf->Get<double>("actor_pose_offset_Y");
        if (sdf->HasElement("actor_pose_offset_Z"))
            this->actor_pose_offset_Z_ = sdf->Get<double>("actor_pose_offset_Z");


        this->target_poses_.clear();
        this->target_poses_.push_back({ 0.0, 0.0, 0.0 });
        this->target_pose_ = this->target_poses_.at(0);

        // ROS2 Subscriptions
        this->vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
            this->vel_topic_, 10,
            std::bind(&ActorCommandSystem::VelCallback, this, std::placeholders::_1));

        this->path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
            this->path_topic_, 10,
            std::bind(&ActorCommandSystem::PathCallback, this, std::placeholders::_1));

        this->animation_sub_ = node_->create_subscription<custom_msgs::msg::ActorAnimation>(
            this->animation_topic_, 10,
            std::bind(&ActorCommandSystem::AnimationCallback, this, std::placeholders::_1));

        // ROS2 Publishers
        this->distance_pub_ = node_->create_publisher<example_interfaces::msg::Float64>(
            this->distance_topic_, rclcpp::QoS(10));

        this->actor_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
            this->actor_pose_topic_, rclcpp::QoS(10));

        this->robot_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
            this->robot_pose_topic_, rclcpp::QoS(10));


        // For multithread callback handling
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
        //RCLCPP_INFO(this->node_->get_logger(), "Recibido cmd_vel: linear x=%.2f, angular z=%.2f", msg->linear.x, msg->angular.z);
        //this->cmd_queue_.emplace(current_linear_vel_, current_angular_vel_);
    }

    ////////////////////////////////////////////////////////////
    void ActorCommandSystem::PathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        this->target_poses_.clear();
        for (const auto& pose_stamped : msg->poses)
        {
            const auto& p = pose_stamped.pose.position;
            const auto& q = pose_stamped.pose.orientation;
            tf2::Quaternion quat(q.x, q.y, q.z, q.w);
            tf2::Matrix3x3 m(quat);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            this->target_poses_.emplace_back(p.x, p.y, yaw);
        }
        this->target_idx_ = 0;
        if (!this->target_poses_.empty())
            this->target_pose_ = this->target_poses_.at(0);
    }

    ////////////////////////////////////////////////////////////
    void ActorCommandSystem::AnimationCallback(const custom_msgs::msg::ActorAnimation::SharedPtr msg)
    {
        this->idle_animation_ = msg->idle;
        this->action_animation_ = msg->action;
    }

    ////////////////////////////////////////////////////////////
    void ActorCommandSystem::ChooseNewTarget()
    {
        if (this->target_idx_ + 1 < this->target_poses_.size())
        {
            this->target_idx_++;
            this->target_pose_ = this->target_poses_.at(this->target_idx_);
        }
        else
        {
            this->target_idx_ = 0;
            this->target_poses_.clear();
            this->target_poses_.push_back({ 0.0, 0.0, 0.0 });
            this->target_pose_ = this->target_poses_.at(0);
        }
    }

    ////////////////////////////////////////////////////////////
    void ActorCommandSystem::PreUpdate(const UpdateInfo& info, EntityComponentManager& ecm)
    {
        //rclcpp::spin_some(this->node_);
        RCLCPP_INFO_THROTTLE(this->node_->get_logger(), *this->node_->get_clock(), 2000, "Modo de seguimiento actual: %s", this->follow_mode_.c_str());


        /* auto poseComp = ecm.Component<gz::sim::components::Pose>(this->actor_entity_);

        if (!poseComp)
            return;

        auto pose = poseComp->Data();

        gz::math::Pose3d newPose = pose; */

        auto traj_ini = ecm.Component<gz::sim::components::TrajectoryPose>(this->actor_entity_);
        auto traj_ini_data = traj_ini->Data();

        gz::math::Pose3d currentPose, newPose;

        newPose.Pos().X() = currentPose.Pos().X() = traj_ini_data.Pos().X();
        newPose.Pos().Y() = currentPose.Pos().Y() = traj_ini_data.Pos().Y();
        newPose.Pos().Z() = currentPose.Pos().Z() = traj_ini_data.Pos().Z();

        //auto rpy = traj_ini_data.Rot().Euler();

        double dt = std::chrono::duration<double>(info.dt).count();


        //RCLCPP_INFO(this->node_->get_logger(), "PoseINI: x=%.5f, y=%.5f, yaw=%.5f", newPose.Pos().X(), newPose.Pos().Y(), newPose.Rot().Euler().Z());
        //RCLCPP_INFO(this->node_->get_logger(), "TRAJINI: x=%.5f, y=%.5f, yaw=%.5f", traj_ini_data.Pos().X(), traj_ini_data.Pos().Y(), traj_ini_data.Rot().Euler().Z());


        // Compute newPose according to follow_mode
        if (this->follow_mode_ == "path")
        {
            auto target2D = gz::math::Vector2d(this->target_pose_.Z(), this->target_pose_.X());
            auto current2D = gz::math::Vector2d(currentPose.Pos().Z(), currentPose.Pos().X());
            auto diff = target2D - current2D;
            double dist = diff.Length();

            if (dist < this->lin_tolerance_)
            {
                this->ChooseNewTarget();
                diff = gz::math::Vector2d(this->target_pose_.Z(), this->target_pose_.X()) - current2D;
            }

            if (diff.Length() > 0)
                diff.Normalize();


            // Calcular yaw objetivo
            double yaw_target = std::atan2(diff.X(), diff.Y()) + this->default_rotation_;
            double yaw_error = yaw_target - this->rotation_pitch_;

            // Normalizar entre [-π, π]
            yaw_error = std::atan2(std::sin(yaw_error), std::cos(yaw_error));

            // Girar suavemente hacia el objetivo
            double yaw_step = std::clamp(yaw_error, -this->ang_velocity_ * dt, this->ang_velocity_ * dt);
            this->rotation_pitch_ += yaw_step;

            // Si ya estamos bien alineados, avanzar
            if (std::abs(yaw_error) < this->ang_tolerance_)
            {
                newPose.Pos().Z() += diff.X() * this->lin_velocity_ * dt;
                newPose.Pos().X() += diff.Y() * this->lin_velocity_ * dt;
            }

            // Aplicar rotación actual
            newPose.Rot() = gz::math::Quaterniond(0, this->rotation_pitch_, 0);
        }
        else if (this->follow_mode_ == "velocity")
        {


            newPose.Pos().Z() += this->current_linear_vel_ * std::cos(this->rotation_pitch_) * dt;
            newPose.Pos().X() += this->current_linear_vel_ * std::sin(this->rotation_pitch_) * dt;

            this->rotation_pitch_ += this->current_angular_vel_ * dt;

            newPose.Rot() = gz::math::Quaterniond(0, this->rotation_pitch_, 0);

            //RCLCPP_INFO(this->node_->get_logger(), "MidPose: x=%.5f, y=%.5f, yaw=%.5f", newPose.Pos().X(), newPose.Pos().Y(), newPose.Rot().Euler().Z());

        }
        else {
            return;
        }

        // 2 Update TrajectoryPose
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


        // CHANGE ANIMATIONS
        //this->animation_counter_++;


        /* if (++this->animation_counter_ >= 8000)
        {
            this->animation_counter_ = 0;

            // Alternar animación
            if (this->desired_animation_ == "idle")
                this->desired_animation_ = "walking";
            else
                this->desired_animation_ = "idle";

            // Cambiar componente
            auto animComp = ecm.Component<components::AnimationName>(this->actor_entity_);
            if (animComp)
                *animComp = components::AnimationName(this->desired_animation_);
            else
                ecm.CreateComponent(this->actor_entity_, components::AnimationName(this->desired_animation_));

            // Marcar cambio
            ecm.SetChanged(this->actor_entity_,
                components::AnimationName::typeId,
                ComponentState::OneTimeChange);

            RCLCPP_INFO(this->node_->get_logger(), "Cambio a animación: %s", this->desired_animation_.c_str());

        } */




        // 3 Update AnimationTime (if not, actor animations is frozen)
        auto animTimeComp = ecm.Component<components::AnimationTime>(this->actor_entity_);
        if (animTimeComp)
        {
            auto currentTime = animTimeComp->Data();
            double distanceTraveled = this->current_linear_vel_ * dt;
            auto updatedTime = currentTime + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(distanceTraveled * this->animation_factor_));


            *animTimeComp = components::AnimationTime(updatedTime);
            ecm.SetChanged(this->actor_entity_, components::AnimationTime::typeId, ComponentState::OneTimeChange);
        }



        //RCLCPP_INFO(this->node_->get_logger(), "Newpose: x=%.5f, y=%.5f, yaw=%.5f", newPose.Pos().X(), newPose.Pos().Y(), newPose.Rot().Euler().Z());
        //RCLCPP_INFO(this->node_->get_logger(), "TRAJPOST: x=%.5f, y=%.5f, yaw=%.5f", trajPost_data.Pos().X(), trajPost_data.Pos().Y(), trajPost_data.Rot().Euler().Z());

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

        auto robotTrajPose = ecm.Component<gz::sim::components::Pose>(this->robot_entity_);
        if (!robotTrajPose) {
            RCLCPP_INFO(this->node_->get_logger(), "Robot Pose component not found");
            return;
        }

        gz::math::Pose3d actorTrajData = actorTrajPose->Data();
        gz::math::Pose3d robotTrajData = robotTrajPose->Data();


        // Compute childWorldPose (robotWorldPose + childPose)
        gz::math::Pose3d childWorldPose;

        if (this->child_link_name_ != "none") {
            auto childTrajPose = ecm.Component<gz::sim::components::Pose>(this->child_entity_);
            auto childTrajData = childTrajPose->Data();
            childWorldPose = robotTrajData * childTrajData;
        }
        else {
            childWorldPose = robotTrajData;
        }



        this->PublishActorPose(actorTrajData);
        this->PublishRobotPose(childWorldPose);
        this->PublishDistance(actorTrajData, childWorldPose);

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

                RCLCPP_INFO(this->node_->get_logger(), "CHILD LINK (%s) FOUND", this->child_link_name_.c_str());
                this->robot_found_ = false;
                return false;
            }
            // CHILD ENTITY NOT KNULL
            this->child_entity_ = child;
            RCLCPP_INFO(this->node_->get_logger(), "CHILD LINK (%s) FOUND", this->child_link_name_.c_str());
        }

        return true;

    }

    void ActorCommandSystem::PublishActorPose(const gz::math::Pose3d& actorTrajData) {

        if (!this->actor_pose_pub_) {
            return;
        }
        geometry_msgs::msg::PoseStamped p;
        p.header.stamp = this->node_->get_clock()->now(); // tiempo ROS
        p.header.frame_id = "world";                      // o "map", según tu sistema

        // Los offset se aplican sobre la coordenada con el mismo nombre del mensaje, pero el Pos() que se mete tiene aplicado el giro
        p.pose.position.x = actorTrajData.Pos().Z() + this->actor_pose_offset_X_;
        p.pose.position.y = actorTrajData.Pos().X() + this->actor_pose_offset_Y_;
        p.pose.position.z = actorTrajData.Pos().Y() + this->actor_pose_offset_Z_;

        p.pose.orientation.x = actorTrajData.Rot().X();
        p.pose.orientation.y = actorTrajData.Rot().Y();
        p.pose.orientation.z = actorTrajData.Rot().Z();
        p.pose.orientation.w = actorTrajData.Rot().W();

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


        // Debido al giro en los ejes del actor, al calcular distancia si que hay que aplicar el offset en el Pos() del mismo nombre.
        actorTrajData.Pos().Z() += this->actor_pose_offset_Z_;
        actorTrajData.Pos().X() += this->actor_pose_offset_X_;
        actorTrajData.Pos().Y() += this->actor_pose_offset_Y_;

        double distance = actorTrajData.Pos().Distance(childWorldPose.Pos());

        example_interfaces::msg::Float64 msg;
        msg.data = distance;
        this->distance_pub_->publish(msg);

    }
}  // namespace ignition_ros2_actor


IGNITION_ADD_PLUGIN(ignition_ros2_actor::ActorCommandSystem,
    ignition::gazebo::System,
    ignition::gazebo::ISystemConfigure,
    ignition::gazebo::ISystemPreUpdate,
    ignition::gazebo::ISystemPostUpdate)

    IGNITION_ADD_PLUGIN_ALIAS(ignition_ros2_actor::ActorCommandSystem,
        "ignition::gazebo::systems::ActorCommandSystem")