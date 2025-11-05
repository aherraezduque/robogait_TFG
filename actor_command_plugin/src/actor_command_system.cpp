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

        /* gz::math::Pose3d newInitPose(
            3.487,   // x
            -3.430,   // y
            0.0,   // z
            1.5708,   // roll
            0.0,   // pitch
            1.5708); // yaw (180 grados)

        if (ecm.EntityHasComponentType(this->actor_entity_, gz::sim::components::Pose::typeId))
        {
            auto poseComp = ecm.Component<gz::sim::components::Pose>(this->actor_entity_);
            *poseComp = gz::sim::components::Pose(newInitPose);
            ecm.SetChanged(this->actor_entity_, gz::sim::components::Pose::typeId,
                gz::sim::ComponentState::OneTimeChange);
        }
        else
        {
            ecm.CreateComponent(this->actor_entity_, gz::sim::components::Pose(newInitPose));
        } */


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
        if (sdf->HasElement("cmd_script_topic"))
            this->script_topic_ = sdf->Get<std::string>("cmd_script_topic");
        // Topics for distance-pose publish
        if (sdf->HasElement("robot_model_name"))
            this->robot_model_name_ = sdf->Get<std::string>("robot_model_name");
        if (sdf->HasElement("child_link_name"))
            this->child_link_name_ = sdf->Get<std::string>("child_link_name");

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

        this->script_sub_ = node_->create_subscription<custom_msgs::msg::ActorTrajectoryPoint>(
            this->script_topic_, 10,
            std::bind(&ActorCommandSystem::ScriptCallback, this, std::placeholders::_1));

        // ROS2 Publishers
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
    void ActorCommandSystem::ScriptCallback(const custom_msgs::msg::ActorTrajectoryPoint::SharedPtr msg) {

        /* tf2::Quaternion quat(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
        tf2::Matrix3x3 m(quat);
        double roll, pitch, yawZ;
        m.getRPY(roll, pitch, yawZ); */

        /* double yawY = yawZ + this->default_rotation_; */


        first_time_ = false;

        TimedWaypoint twp;
        twp.x = msg->pose.position.x;
        twp.y = msg->pose.position.y;
        twp.z = msg->pose.position.z;
        //twp.yaw = yawY;
        twp.yaw = msg->pose.orientation.z;
        twp.t = msg->t;

        this->script_path_mutex_.lock();

        if (msg->clear || !this->defined_script_path_) {
            this->script_path_.clear();
            this->defined_script_path_ = true;

            //this->path_start_time_ = this->sim_time_sec_ -twp.t;
            this->have_tramo_ = false;
            this->current_tramo_ = Tramo{};
            this->timed_idx_ = 0;

        }
        this->script_path_.push_back(twp);

        this->script_path_mutex_.unlock();

        RCLCPP_INFO(this->node_->get_logger(),
            "New waypoint: (%.3f, %.3f, %.3f), yaw = %.3f rad, t= %.3f, clear = %d ",
            twp.x, twp.y, twp.z, twp.yaw, twp.t, int(msg->clear));



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
        /* if(_info.pause){
            return;
        } */
        //rclcpp::spin_some(this->node_);
        //RCLCPP_INFO_THROTTLE(this->node_->get_logger(), *this->node_->get_clock(), 2000, "Modo de seguimiento actual: %s", this->follow_mode_.c_str());




        auto traj_ini = ecm.Component<gz::sim::components::TrajectoryPose>(this->actor_entity_);
        auto traj_ini_data = traj_ini->Data();

        gz::math::Pose3d currentPose, newPose;

        newPose.Pos().X() = currentPose.Pos().X() = traj_ini_data.Pos().X();
        newPose.Pos().Y() = currentPose.Pos().Y() = traj_ini_data.Pos().Y();
        newPose.Pos().Z() = currentPose.Pos().Z() = traj_ini_data.Pos().Z();



        //auto rpy = traj_ini_data.Rot().Euler();

        double dt = std::chrono::duration<double>(info.dt).count();


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
        if (this->follow_mode_ == "velocity")
        {


            newPose.Pos().Z() += this->current_linear_vel_ * std::cos(this->rotation_pitch_) * dt;
            newPose.Pos().X() += this->current_linear_vel_ * std::sin(this->rotation_pitch_) * dt;

            this->rotation_pitch_ += this->current_angular_vel_ * dt;

            newPose.Rot() = gz::math::Quaterniond(0, this->rotation_pitch_, 0);

        }
        if (this->follow_mode_ == "script")
        {
            if (first_time_) {

                newPose.Rot() = gz::math::Quaterniond(0, this->default_rotation_, 0);
            }
            else {
                newPose.Rot() = gz::math::Quaterniond(0, this->default_rotation_, 0);
                AdvanceScriptVelBased(dt, newPose);

                // Update tramo if path hasnt ended
                if (!have_tramo_ && script_path_.size() > timed_idx_ + 1)
                {
                    StartNewTramo(script_path_[timed_idx_], script_path_[timed_idx_ + 1]);
                    timed_idx_++;
                }

            }

        }
        /* else
        {
            return;
        } */

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


        // 3 Update AnimationTime (if not, actor animations is frozen)
        /* auto animTimeComp = ecm.Component<components::AnimationTime>(this->actor_entity_);
        if (animTimeComp)
        {
            auto currentTime = animTimeComp->Data();
            double distanceTraveled = this->current_linear_vel_ * dt;
            auto updatedTime = currentTime + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(distanceTraveled * this->animation_factor_));


            *animTimeComp = components::AnimationTime(updatedTime);
            ecm.SetChanged(this->actor_entity_, components::AnimationTime::typeId, ComponentState::OneTimeChange);
        } */

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

    void ActorCommandSystem::StartNewTramo(const TimedWaypoint& A, const TimedWaypoint& B) {

        double dx = B.y - A.y;
        double dz = B.x - A.x;
        double dist = std::hypot(dx, dz);
        double seg_dt = (B.t - A.t);

        this->rotation_pitch_ = A.yaw;
        current_tramo_.A = A;
        current_tramo_.B = B;

        // Compute linear vel and direction in this "tramo"
        current_tramo_.linear_vel = dist / seg_dt;
        //current_tramo_.yaw_motion = this->rotation_pitch_ + std::atan2(dx, dz);
        current_tramo_.yaw_motion = std::atan2(dx, dz);
        RCLCPP_INFO(this->node_->get_logger(), "yaw motion %lf", current_tramo_.yaw_motion);


        // Angular speed for smooth visual
        double dYaw = ShortestAngle(A.yaw, B.yaw);
        current_tramo_.angular_vel_visual = dYaw / seg_dt;

        // nº of steps = seg_dt / dt (0.001 Gazebo Ignition)
        current_tramo_.steps_remaining = static_cast<int>(std::round(seg_dt / 0.001));

        have_tramo_ = true;
    }

    void ActorCommandSystem::AdvanceScriptVelBased(double dt, gz::math::Pose3d& pose) {


        if (current_tramo_.steps_remaining > 0) {

            // Compute dz and dx 
            pose.Pos().Z() += std::cos(current_tramo_.yaw_motion) * current_tramo_.linear_vel * dt;
            pose.Pos().X() += std::sin(current_tramo_.yaw_motion) * current_tramo_.linear_vel * dt;

            /* RCLCPP_INFO(this->node_->get_logger(), "x: %f y: %f, current_tramo_yaw_motion : %f , curr_linear_vel: %f",
                pose.Pos().Z(), pose.Pos().X(), current_tramo_.yaw_motion, current_tramo_.linear_vel); */
                // Compute new rotation_pitch
            this->rotation_pitch_ += current_tramo_.angular_vel_visual * dt;
            pose.Rot() = gz::math::Quaterniond(0, this->rotation_pitch_, 0);

            // For animation movement update
            current_linear_vel_ = current_tramo_.linear_vel;
            //current_angular_vel_ = current_tramo_.angular_vel_visual;

            current_tramo_.steps_remaining--;

        }
        else {
            // You have reached point B. 
            /* pose.Pos().Z() = current_tramo_.B.x;
            pose.Pos().X() = current_tramo_.B.y;

            rotation_pitch_ = current_tramo_.B.yaw;
            pose.Rot() = gz::math::Quaterniond(0, this->rotation_pitch_, 0); */

            /* RCLCPP_INFO(this->node_->get_logger(), "x: %f y: %f, current_tramo_yaw_motion : %f , curr_linear_vel: %f",
                pose.Pos().Z(), pose.Pos().X(), current_tramo_.yaw_motion, current_tramo_.linear_vel); */

                /* current_linear_vel_ = 0.0;
                current_angular_vel_ = 0.0; */

            current_linear_vel_ = 0.0;
            current_angular_vel_ = 0.0;

            have_tramo_ = false;
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


        gz::math::Pose3d actor_Position = actorTrajData;

        /* RCLCPP_INFO(
            this->node_->get_logger(),
            "ANTES: Actor pos: (%.2f, %.2f, %.2f), Child pos: (%.2f, %.2f, %.2f)",
            actorTrajData.Pos().X(), actorTrajData.Pos().Y(), actorTrajData.Pos().Z(),
            childWorldPose.Pos().X(), childWorldPose.Pos().Y(), childWorldPose.Pos().Z()
        ); */

        // Debido al giro en los ejes del actor, al calcular distancia si que hay que aplicar el offset en el Pos() del mismo nombre.
        /* actorTrajData.Pos().Z() += this->actor_pose_offset_Z_;
        actorTrajData.Pos().X() += this->actor_pose_offset_X_;
        actorTrajData.Pos().Y() += this->actor_pose_offset_Y_; */




        actor_Position.Pos().X() = actorTrajData.Pos().Z() + this->actor_pose_offset_X_;
        actor_Position.Pos().Y() = actorTrajData.Pos().X() + this->actor_pose_offset_Y_;
        actor_Position.Pos().Z() = actorTrajData.Pos().Y() + this->actor_pose_offset_Z_;


        //double distance = actorTrajData.Pos().Distance(childWorldPose.Pos());
        double distance = actor_Position.Pos().Distance(childWorldPose.Pos());



        /*  double distance1 = (actorTrajData.Pos().X() - childWorldPose.Pos().X()) * (actorTrajData.Pos().X() - childWorldPose.Pos().X());
         double distance2 = (actorTrajData.Pos().Y() - childWorldPose.Pos().Y()) * (actorTrajData.Pos().Y() - childWorldPose.Pos().Y());
         double distance3 = (actorTrajData.Pos().Z() - childWorldPose.Pos().Z()) * (actorTrajData.Pos().Z() - childWorldPose.Pos().Z());

         double distance4 = sqrt(distance1 + distance2 + distance3);
         RCLCPP_INFO(
             this->node_->get_logger(),
             "DESPUES: Actor pos: (%.2f, %.2f, %.2f), Child pos: (%.2f, %.2f, %.2f), distance calculada = %.2f la otra = %.2f",
             actorTrajData.Pos().X(), actorTrajData.Pos().Y(), actorTrajData.Pos().Z(),
             childWorldPose.Pos().X(), childWorldPose.Pos().Y(), childWorldPose.Pos().Z(),
             distance4, distance
         );
  */



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