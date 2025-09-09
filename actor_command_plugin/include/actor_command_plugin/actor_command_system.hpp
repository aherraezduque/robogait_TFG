#ifndef ACTOR_COMMAND_SYSTEM_HPP
#define ACTOR_COMMAND_SYSTEM_HPP

#include <memory>
#include <string>
#include <queue>
#include <vector>

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Actor.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Model.hh>

#include <gz/math/Vector3.hh>
#include <gz/math/Pose3.hh>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <custom_msgs/msg/actor_animation.hpp>
#include <example_interfaces/msg/float64.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>


namespace ignition_ros2_actor
{

    class ActorCommandSystem
        : public ignition::gazebo::System,
        public ignition::gazebo::ISystemConfigure,
        public ignition::gazebo::ISystemPreUpdate,
        public ignition::gazebo::ISystemPostUpdate
    {
    public:
        ActorCommandSystem();
        ~ActorCommandSystem() override;

        void Configure(const gz::sim::Entity& entity,
            const std::shared_ptr<const sdf::Element>& sdf,
            gz::sim::EntityComponentManager& ecm,
            gz::sim::EventManager& eventMgr) override;

        void PreUpdate(const gz::sim::UpdateInfo& info,
            gz::sim::EntityComponentManager& ecm) override;

        void PostUpdate(const gz::sim::UpdateInfo& info,
            const gz::sim::EntityComponentManager& ecm) override;
    private:
        // ROS 2 Node
        std::shared_ptr<rclcpp::Node> node_;

        // Multi-threaded ROS executor
        std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
        std::thread ros_spin_thread_;

        // Topic config //
        // Subscriptions topics names
        std::string vel_topic_ = "/actor_cmd_vel";
        std::string path_topic_ = "/actor_cmd_path";
        std::string animation_topic_ = "/actor_cmd_animation";
        // Publishers topics names
        std::string distance_topic_ = "/actor_robot/distance";
        std::string actor_pose_topic_ = "/actor_robot/actor_pose";
        std::string robot_pose_topic_ = "/actor_robot/robot_pose";

        //Cmd subscriptions
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
        rclcpp::Subscription<custom_msgs::msg::ActorAnimation>::SharedPtr animation_sub_;

        // Distance-pose publishers 
        rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr distance_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr actor_pose_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_pub_;

        // Control mode
        std::string follow_mode_ = "none";

        // Velocity command storage
        double current_linear_vel_ = 0.0;
        double current_angular_vel_ = 0.0;
        //std::queue<std::pair<double, double>> cmd_queue_;

        // Path following
        std::vector<gz::math::Vector3d> target_poses_;
        gz::math::Vector3d target_pose_;
        std::size_t target_idx_ = 0;

        // Tolerances
        double lin_tolerance_ = 0.1;
        double ang_tolerance_ = 0.1;
        double lin_velocity_ = 1.0;
        double ang_velocity_ = 0.5;

        bool orientation_fixed_ = false;

        // Animations 
        std::string idle_animation_ = "idle";
        std::string action_animation_ = "walking";
        std::string desired_animation_ = "idle";
        double animation_factor_ = 4.0;
        int animation_counter_ = 0;

        // Default rotation (yaw offset)
        double default_rotation_ = 0.0;

        // Actor reference
        gz::sim::Entity actor_entity_;
        bool initialized_ = false;
        double actor_pose_offset_X_ = 0.0;
        double actor_pose_offset_Y_ = 0.0;
        double actor_pose_offset_Z_ = 0.0;


        double rotation_pitch_ = 0.0;

        // Robot model name for distance publishing
        std::string robot_model_name_ = "rover_mini";
        std::string child_link_name_ = "none";

        gz::sim::Entity robot_entity_ = gz::sim::kNullEntity;
        gz::sim::Entity child_entity_ = gz::sim::kNullEntity;

        bool robot_found_ = false;

        // Callbacks cmd topics 
        void VelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
        void PathCallback(const nav_msgs::msg::Path::SharedPtr msg);
        void AnimationCallback(const custom_msgs::msg::ActorAnimation::SharedPtr msg);

        // Helpers
        void ChooseNewTarget();

        bool CheckEntitiesFound(const gz::sim::EntityComponentManager& ecm);

        void PublishActorPose(const gz::math::Pose3d& actorTrajData);
        void PublishRobotPose(const gz::math::Pose3d& childWorldPose);
        void PublishDistance(gz::math::Pose3d& actorTrajData, const gz::math::Pose3d& childWorldPose);

    };

}  // namespace ignition_ros2_actor

#endif  // ACTOR_COMMAND_SYSTEM_HPP
