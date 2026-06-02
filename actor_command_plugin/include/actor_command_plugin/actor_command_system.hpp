#ifndef ACTOR_COMMAND_SYSTEM_HPP
#define ACTOR_COMMAND_SYSTEM_HPP

#include <memory>
#include <string>
#include <queue>
#include <vector>
#include <cmath>

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
#include <custom_msgs/msg/actor_trajectory_point.hpp>   
#include <std_msgs/msg/float64.hpp>
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
        std::string script_topic_ = "/actor_cmd_script";
        // Publishers topics names
        std::string distance_topic_ = "/actor_robot/distance";
        std::string actor_pose_topic_ = "/actor_robot/actor_pose";
        std::string robot_pose_topic_ = "/actor_robot/robot_pose";

        bool enable_distance_topic_ = false;
        bool enable_actor_pose_topic_ = false;
        bool enable_robot_pose_topic_ = false;

        bool first_time_ = true;

        // Cmd subscriptions
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
        rclcpp::Subscription<custom_msgs::msg::ActorTrajectoryPoint>::SharedPtr script_sub_;
        // Distance-pose publishers 
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr distance_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr actor_pose_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_pub_;

        // Control mode
        std::string follow_mode_ = "none";

        // Velocity command storage
        double current_linear_vel_ = 0.0;               // for cmd_vel mode and animation update
        double current_angular_vel_ = 0.0;              // for cmd_vel mode and animation update
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


        // Animations 
        std::string action_animation_ = "walking";
        double animation_factor_ = 0.0;

        // Default rotation (yaw offset)
        double default_rotation_ = 0.0;

        // Script path variables
        struct TimedWaypoint {
            double x, y, z;     // Actor pose
            double yaw;         // ROT in Y due to mixamo and gazebo axis reference
            double t;           // Time
        };

        struct Tramo {
            TimedWaypoint A, B;
            double linear_vel = 0.0;
            double yaw_motion = 0.0;
            double angular_vel_visual = 0.0;
            int steps_remaining = 0;
        };

        Tramo current_tramo_;
        bool have_tramo_ = false;

        std::vector<TimedWaypoint> script_path_;    // Path defined by TimedWaypoints
        size_t timed_idx_ = 0;                      // Path index
        std::mutex script_path_mutex_;
        bool defined_script_path_ = false;          // If true, a path has been already defined
        //double path_start_time_ = 0.00;             // Local script_path time in s



        // Actor reference
        gz::sim::Entity actor_entity_;
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

        // Helpers Configure
        void EnsureActorComponents(gz::sim::EntityComponentManager& ecm);
        void InitRosNode();
        void LoadSdfParameters(const std::shared_ptr<const sdf::Element>& sdf);
        void InitializePathTarget();
        void CreateRosSubscriptions();
        void CreateRosPublishers();
        void StartRosExecutor();

        // Callbacks cmd topics 
        void VelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
        void PathCallback(const nav_msgs::msg::Path::SharedPtr msg);
        void ScriptCallback(const custom_msgs::msg::ActorTrajectoryPoint::SharedPtr msg);

        // Helpers
        void ChooseNewTarget();

        void StartNewTramo(const TimedWaypoint& A, const TimedWaypoint& B, double dt);         // Script path
        void AdvanceScriptVelBased(double dt, gz::math::Pose3d& pose);                          // Script path

        bool CheckEntitiesFound(const gz::sim::EntityComponentManager& ecm);

        gz::math::Pose3d GetActorWorldPose(const gz::math::Pose3d& actorPose);

        void PublishActorPose(const gz::math::Pose3d& actorTrajData);
        void PublishRobotPose(const gz::math::Pose3d& childWorldPose);
        void PublishDistance(gz::math::Pose3d& actorTrajData, const gz::math::Pose3d& childWorldPose);

        double ShortestAngle(double from, double to);

    };

}  // namespace ignition_ros2_actor

#endif  // ACTOR_COMMAND_SYSTEM_HPP
