/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "diff_drive_second_order/DiffDrive_second_order.hh"

#include <gz/msgs/odometry.pb.h>

#include <limits>
#include <mutex>
#include <set>
#include <string>
#include <vector>
#include <deque>

#include <gz/common/Profiler.hh>
#include <gz/math/DiffDriveOdometry.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/SpeedLimiter.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/components/CanonicalLink.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/JointVelocityCmd.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

#include <iostream>
#include <vector>

using namespace gz;
using namespace gz::sim;
using namespace systems;

/// \brief Velocity command.
struct Commands
{
  /// \brief Linear velocity.
  double lin;

  /// \brief Angular velocity.
  double ang;

  Commands() : lin(0.0), ang(0.0) {}
};

/// \brief Second order filter state for velocity dynamics
struct SecondOrderFilterState
{
  /// \brief Poly order
  std::size_t order = 2;

  /// \brief State variable previous values
  std::vector<double> y_hist;


  /// \brief Input previous values
  std::vector<double> u_hist;

  SecondOrderFilterState() : y_hist(order + 1, 0.000), u_hist(order + 1, 0.000) {}
};


class ignition::gazebo::systems::DiffDrive_second_order_Private
{
  /// \brief Callback for velocity subscription
  /// \param[in] _msg Velocity message
public: void OnCmdVel(const msgs::Twist& _msg);

      /// \brief Callback for enable/disable subscription
      /// \param[in] _msg Boolean message
public: void OnEnable(const msgs::Boolean& _msg);

      /// \brief Update odometry and publish an odometry message.
      /// \param[in] _info System update information.
      /// \param[in] _ecm The EntityComponentManager of the given simulation
      /// instance.
public: void UpdateOdometry(const UpdateInfo& _info,
  const EntityComponentManager& _ecm);

      /// \brief Update the linear and angular velocities.
      /// \param[in] _info System update information.
      /// \param[in] _ecm The EntityComponentManager of the given simulation
      /// instance.
public: void UpdateVelocity(const UpdateInfo& _info,
  const EntityComponentManager& _ecm);

      /// \brief Apply second order filter to velocity commands
      /// \param[in] _input Input velocity command
      /// \param[in] _filter_state Filter state to update
      /// \param[in] _dt Time step
      /// \return Filtered velocity
public: double ApplySecondOrderFilter(double _input,
  SecondOrderFilterState& _filter_state, double _dt);

      /// \brief Apply delay to input velocity commands
      /// \param[in] _input 
      /// \param[in] _buffer 
      /// \param[in] _delaySamples 
      /// \return Delayed input velocity
public: double ApplyPureDelay(double _input,
  std::deque<double>& _buffer, std::size_t _delaySamples);

      /// \brief Ignition communication node.
public: transport::Node node;

      /// \brief Entity of the left joint
public: std::vector<Entity> leftJoints;

      /// \brief Entity of the right joint
public: std::vector<Entity> rightJoints;

      /// \brief Name of left joint
public: std::vector<std::string> leftJointNames;

      /// \brief Name of right joint
public: std::vector<std::string> rightJointNames;

      /// \brief Calculated speed of left joint
public: double leftJointSpeed{ 0 };

      /// \brief Calculated speed of right joint
public: double rightJointSpeed{ 0 };

      /// \brief Distance between wheels
public: double wheelSeparation{ 1.0 };

      /// \brief Wheel radius
public: double wheelRadius{ 0.2 };

      /// \brief Model interface
public: Model model{ kNullEntity };

      /// \brief The model's canonical link.
public: Link canonicalLink{ kNullEntity };

      /// \brief Update period calculated from <odom__publish_frequency>.
public: std::chrono::steady_clock::duration odomPubPeriod{ 0 };

      /// \brief Last sim time odom was published.
public: std::chrono::steady_clock::duration lastOdomPubTime{ 0 };

      /// \brief Diff drive odometry.
public: math::DiffDriveOdometry odom;

      /// \brief Diff drive odometry message publisher.
public: transport::Node::Publisher odomPub;

      /// \brief Diff drive tf message publisher.
public: transport::Node::Publisher tfPub;

      /// \brief Linear velocity limiter.
public: std::unique_ptr<math::SpeedLimiter> limiterLin;

      /// \brief Angular velocity limiter.
public: std::unique_ptr<math::SpeedLimiter> limiterAng;

      /// \brief Previous control command.
public: Commands last0Cmd;

      /// \brief Previous control command to last0Cmd.
public: Commands last1Cmd;

      /// \brief Last target velocity requested.
public: msgs::Twist targetVel;

      /// \brief Enable/disable state of the controller.
public: bool enabled;

      /// \brief A mutex to protect the target velocity command.
public: std::mutex mutex;

      /// \brief frame_id from sdf.
public: std::string sdfFrameId;

      /// \brief child_frame_id from sdf.
public: std::string sdfChildFrameId;



      /// \brief Enable second order filter
public: bool enableSecondOrderFilter{ false };

      /// \brief Numerator b_0 coeficient
public: double b_0_{ 0.002294264339152 };

      /// \brief Numerator b_1 coeficient
public: double b_1_{ 0.002294264339152 };

      /// \brief Numerator b_2 coeficient
public: double b_2_{ 0.00 };

      /// \brief Denominator a_0 coeficient
public: double a_0_{ 1.00 };

      /// \brief Denominator a_1 coeficient
public: double a_1_{ -0.995012468827930 };

      /// \brief Denominator a_2 coeficient
public: double a_2_{ 0.00 };

      /// \brief Linear velocity filter state
public: SecondOrderFilterState linearFilterState;

      /// \brief Angular velocity filter state
public: SecondOrderFilterState angularFilterState;

public: int steps_counter{ 0 };

      /// \brief Enable pure delay
public: bool enablePureDelay{ false };

      /// \brief Pure Delay in seconds
public: double pureDelaySeconds{ 0.0 };

      /// \brief Buffer for linear delay
public: std::deque<double> linearDelayBuffer;

      /// \brief Buffer for angular delay
public: std::deque<double> angularDelayBuffer;

};


//////////////////////////////////////////////////
DiffDrive_second_order::DiffDrive_second_order()
  : dataPtr(std::make_unique<DiffDrive_second_order_Private>())
{
}

//////////////////////////////////////////////////
void DiffDrive_second_order::Configure(const Entity& _entity,
  const std::shared_ptr<const sdf::Element>& _sdf,
  EntityComponentManager& _ecm,
  EventManager&/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);

  // Get the canonical link
  std::vector<Entity> links = _ecm.ChildrenByComponents(
    this->dataPtr->model.Entity(), components::CanonicalLink());
  if (!links.empty())
    this->dataPtr->canonicalLink = Link(links[0]);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    ignerr << "DiffDrive plugin should be attached to a model entity. "
      << "Failed to initialize." << std::endl;
    return;
  }

  // Ugly, but needed because the sdf::Element::GetElement is not a const
  // function and _sdf is a const shared pointer to a const sdf::Element.
  auto ptr = const_cast<sdf::Element*>(_sdf.get());

  // Get params from SDF
  sdf::ElementPtr sdfElem = ptr->GetElement("left_joint");
  while (sdfElem)
  {
    this->dataPtr->leftJointNames.push_back(sdfElem->Get<std::string>());
    sdfElem = sdfElem->GetNextElement("left_joint");
  }
  sdfElem = ptr->GetElement("right_joint");
  while (sdfElem)
  {
    this->dataPtr->rightJointNames.push_back(sdfElem->Get<std::string>());
    sdfElem = sdfElem->GetNextElement("right_joint");
  }

  this->dataPtr->wheelSeparation = _sdf->Get<double>("wheel_separation",
    this->dataPtr->wheelSeparation).first;
  this->dataPtr->wheelRadius = _sdf->Get<double>("wheel_radius",
    this->dataPtr->wheelRadius).first;

  // Second order filter parameters
  this->dataPtr->enableSecondOrderFilter = _sdf->Get<bool>("enable_second_order_filter",
    this->dataPtr->enableSecondOrderFilter).first;
  // Numerator poly coeficient
  this->dataPtr->b_0_ = _sdf->Get<double>("b_0",
    this->dataPtr->b_0_).first;
  this->dataPtr->b_1_ = _sdf->Get<double>("b_1",
    this->dataPtr->b_1_).first;
  this->dataPtr->b_2_ = _sdf->Get<double>("b_2",
    this->dataPtr->b_2_).first;
  // Denominator poly coeficient a_0 is always 1.0
  this->dataPtr->a_0_ = _sdf->Get<double>("a_0",
    this->dataPtr->a_0_).first;
  this->dataPtr->a_1_ = _sdf->Get<double>("a_1",
    this->dataPtr->a_1_).first;
  this->dataPtr->a_2_ = _sdf->Get<double>("a_2",
    this->dataPtr->a_2_).first;
  // Pure Delay 
  this->dataPtr->enablePureDelay = _sdf->Get<bool>("enable_pure_delay",
    this->dataPtr->enablePureDelay).first;
  this->dataPtr->pureDelaySeconds = _sdf->Get<double>("pure_delay_seconds",
    this->dataPtr->pureDelaySeconds).first;



  if (this->dataPtr->enableSecondOrderFilter)
  {

    std::cout << "DiffDrive: Second order filter enabled with parameters: bo: "
      << this->dataPtr->b_0_ << "  b1: " << this->dataPtr->b_1_ << "  b2: " << this->dataPtr->b_2_ <<
      "  ao: " << this->dataPtr->a_0_ << "  a1: " << this->dataPtr->a_1_ << "  a2: " << this->dataPtr->a_2_ << std::endl;

    if (this->dataPtr->enablePureDelay)
    {
      std::cout << "with delay:" << this->dataPtr->pureDelaySeconds << " s" << std::endl;
    }

  }


  // Instantiate the speed limiters.
  this->dataPtr->limiterLin = std::make_unique<math::SpeedLimiter>();
  this->dataPtr->limiterAng = std::make_unique<math::SpeedLimiter>();

  // Parse speed limiter parameters.

  // Min Velocity
  if (_sdf->HasElement("min_velocity"))
  {
    const double minVel = _sdf->Get<double>("min_velocity");
    this->dataPtr->limiterLin->SetMinVelocity(minVel);
    this->dataPtr->limiterAng->SetMinVelocity(minVel);
  }
  if (_sdf->HasElement("min_linear_velocity"))
  {
    const double minLinVel = _sdf->Get<double>("min_linear_velocity");
    this->dataPtr->limiterLin->SetMinVelocity(minLinVel);
  }
  if (_sdf->HasElement("min_angular_velocity"))
  {
    const double minAngVel = _sdf->Get<double>("min_angular_velocity");
    this->dataPtr->limiterAng->SetMinVelocity(minAngVel);
  }

  // Max Velocity
  if (_sdf->HasElement("max_velocity"))
  {
    const double maxVel = _sdf->Get<double>("max_velocity");
    this->dataPtr->limiterLin->SetMaxVelocity(maxVel);
    this->dataPtr->limiterAng->SetMaxVelocity(maxVel);
  }
  if (_sdf->HasElement("max_linear_velocity"))
  {
    const double maxLinVel = _sdf->Get<double>("max_linear_velocity");
    this->dataPtr->limiterLin->SetMaxVelocity(maxLinVel);
  }
  if (_sdf->HasElement("max_angular_velocity"))
  {
    const double maxAngVel = _sdf->Get<double>("max_angular_velocity");
    this->dataPtr->limiterAng->SetMaxVelocity(maxAngVel);
  }

  // Min Acceleration
  if (_sdf->HasElement("min_acceleration"))
  {
    const double minAccel = _sdf->Get<double>("min_acceleration");
    this->dataPtr->limiterLin->SetMinAcceleration(minAccel);
    this->dataPtr->limiterAng->SetMinAcceleration(minAccel);
  }
  if (_sdf->HasElement("min_linear_acceleration"))
  {
    const double minLinAccel = _sdf->Get<double>("min_linear_acceleration");
    this->dataPtr->limiterLin->SetMinAcceleration(minLinAccel);
  }
  if (_sdf->HasElement("min_angular_acceleration"))
  {
    const double minAngAccel = _sdf->Get<double>("min_angular_acceleration");
    this->dataPtr->limiterAng->SetMinAcceleration(minAngAccel);
  }

  // Max Acceleration
  if (_sdf->HasElement("max_acceleration"))
  {
    const double maxAccel = _sdf->Get<double>("max_acceleration");
    this->dataPtr->limiterLin->SetMaxAcceleration(maxAccel);
    this->dataPtr->limiterAng->SetMaxAcceleration(maxAccel);
  }
  if (_sdf->HasElement("max_linear_acceleration"))
  {
    const double maxLinAccel = _sdf->Get<double>("max_linear_acceleration");
    this->dataPtr->limiterLin->SetMaxAcceleration(maxLinAccel);
  }
  if (_sdf->HasElement("max_angular_acceleration"))
  {
    const double maxAngAccel = _sdf->Get<double>("max_angular_acceleration");
    this->dataPtr->limiterAng->SetMaxAcceleration(maxAngAccel);
  }

  // Min Jerk
  if (_sdf->HasElement("min_jerk"))
  {
    const double minJerk = _sdf->Get<double>("min_jerk");
    this->dataPtr->limiterLin->SetMinJerk(minJerk);
    this->dataPtr->limiterAng->SetMinJerk(minJerk);
  }
  if (_sdf->HasElement("min_linear_jerk"))
  {
    const double minLinJerk = _sdf->Get<double>("min_linear_jerk");
    this->dataPtr->limiterLin->SetMinJerk(minLinJerk);
  }
  if (_sdf->HasElement("min_angular_jerk"))
  {
    const double minAngJerk = _sdf->Get<double>("min_angular_jerk");
    this->dataPtr->limiterAng->SetMinJerk(minAngJerk);
  }

  // Max Jerk
  if (_sdf->HasElement("max_jerk"))
  {
    const double maxJerk = _sdf->Get<double>("max_jerk");
    this->dataPtr->limiterLin->SetMaxJerk(maxJerk);
    this->dataPtr->limiterAng->SetMaxJerk(maxJerk);
  }
  if (_sdf->HasElement("max_linear_jerk"))
  {
    const double maxLinJerk = _sdf->Get<double>("max_linear_jerk");
    this->dataPtr->limiterLin->SetMaxJerk(maxLinJerk);
  }
  if (_sdf->HasElement("max_angular_jerk"))
  {
    const double maxAngJerk = _sdf->Get<double>("max_angular_jerk");
    this->dataPtr->limiterAng->SetMaxJerk(maxAngJerk);
  }

  double odomFreq = _sdf->Get<double>("odom_publish_frequency", 50).first;
  if (odomFreq > 0)
  {
    std::chrono::duration<double> odomPer{ 1 / odomFreq };
    this->dataPtr->odomPubPeriod =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(odomPer);
  }

  // Setup odometry.
  this->dataPtr->odom.SetWheelParams(this->dataPtr->wheelSeparation,
    this->dataPtr->wheelRadius, this->dataPtr->wheelRadius);

  // Subscribe to commands
  std::vector<std::string> topics;
  if (_sdf->HasElement("topic"))
  {
    topics.push_back(_sdf->Get<std::string>("topic"));
  }
  topics.push_back("/model/" + this->dataPtr->model.Name(_ecm) + "/cmd_vel");
  auto topic = validTopic(topics);

  this->dataPtr->node.Subscribe(topic, &DiffDrive_second_order_Private::OnCmdVel,
    this->dataPtr.get());

  // Subscribe to enable/disable
  std::vector<std::string> enableTopics;
  enableTopics.push_back(
    "/model/" + this->dataPtr->model.Name(_ecm) + "/enable");
  auto enableTopic = validTopic(enableTopics);

  if (!enableTopic.empty())
  {
    this->dataPtr->node.Subscribe(enableTopic, &DiffDrive_second_order_Private::OnEnable,
      this->dataPtr.get());
  }
  this->dataPtr->enabled = true;

  std::vector<std::string> odomTopics;
  if (_sdf->HasElement("odom_topic"))
  {
    odomTopics.push_back(_sdf->Get<std::string>("odom_topic"));
  }
  odomTopics.push_back("/model/" + this->dataPtr->model.Name(_ecm) +
    "/odometry");
  auto odomTopic = validTopic(odomTopics);

  this->dataPtr->odomPub = this->dataPtr->node.Advertise<msgs::Odometry>(
    odomTopic);

  std::string tfTopic{ "/model/" + this->dataPtr->model.Name(_ecm) +
    "/tf" };
  if (_sdf->HasElement("tf_topic"))
    tfTopic = _sdf->Get<std::string>("tf_topic");
  this->dataPtr->tfPub = this->dataPtr->node.Advertise<msgs::Pose_V>(
    tfTopic);

  if (_sdf->HasElement("frame_id"))
    this->dataPtr->sdfFrameId = _sdf->Get<std::string>("frame_id");

  if (_sdf->HasElement("child_frame_id"))
    this->dataPtr->sdfChildFrameId = _sdf->Get<std::string>("child_frame_id");

  ignmsg << "DiffDrive subscribing to twist messages on [" << topic << "]"
    << std::endl;
}

//////////////////////////////////////////////////
void DiffDrive_second_order::PreUpdate(const UpdateInfo& _info,
  EntityComponentManager& _ecm)
{
  IGN_PROFILE("DiffDrive::PreUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    ignwarn << "Detected jump back in time ["
      << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
      << "s]. System may not work properly." << std::endl;
  }

  // If the joints haven't been identified yet, look for them
  static std::set<std::string> warnedModels;
  auto modelName = this->dataPtr->model.Name(_ecm);
  if (this->dataPtr->leftJoints.empty() ||
    this->dataPtr->rightJoints.empty())
  {
    bool warned{ false };
    for (const std::string& name : this->dataPtr->leftJointNames)
    {
      Entity joint = this->dataPtr->model.JointByName(_ecm, name);
      if (joint != kNullEntity)
        this->dataPtr->leftJoints.push_back(joint);
      else if (warnedModels.find(modelName) == warnedModels.end())
      {
        ignwarn << "Failed to find left joint [" << name << "] for model ["
          << modelName << "]" << std::endl;
        warned = true;
      }
    }

    for (const std::string& name : this->dataPtr->rightJointNames)
    {
      Entity joint = this->dataPtr->model.JointByName(_ecm, name);
      if (joint != kNullEntity)
        this->dataPtr->rightJoints.push_back(joint);
      else if (warnedModels.find(modelName) == warnedModels.end())
      {
        ignwarn << "Failed to find right joint [" << name << "] for model ["
          << modelName << "]" << std::endl;
        warned = true;
      }
    }
    if (warned)
    {
      warnedModels.insert(modelName);
    }
  }

  if (this->dataPtr->leftJoints.empty() || this->dataPtr->rightJoints.empty())
    return;

  if (warnedModels.find(modelName) != warnedModels.end())
  {
    ignmsg << "Found joints for model [" << modelName
      << "], plugin will start working." << std::endl;
    warnedModels.erase(modelName);
  }

  // Nothing left to do if paused.
  if (_info.paused)
    return;

  for (Entity joint : this->dataPtr->leftJoints)
  {
    // skip this entity if it has been removed
    if (!_ecm.HasEntity(joint))
      continue;

    // Update wheel velocity
    auto vel = _ecm.Component<components::JointVelocityCmd>(joint);

    if (vel == nullptr)
    {
      _ecm.CreateComponent(
        joint, components::JointVelocityCmd({ this->dataPtr->leftJointSpeed }));
    }
    else
    {
      *vel = components::JointVelocityCmd({ this->dataPtr->leftJointSpeed });
    }
  }

  for (Entity joint : this->dataPtr->rightJoints)
  {
    // skip this entity if it has been removed
    if (!_ecm.HasEntity(joint))
      continue;

    // Update wheel velocity
    auto vel = _ecm.Component<components::JointVelocityCmd>(joint);

    if (vel == nullptr)
    {
      _ecm.CreateComponent(joint,
        components::JointVelocityCmd({ this->dataPtr->rightJointSpeed }));
    }
    else
    {
      *vel = components::JointVelocityCmd({ this->dataPtr->rightJointSpeed });
    }
  }

  // Create the left and right side joint position components if they
  // don't exist.
  auto leftPos = _ecm.Component<components::JointPosition>(
    this->dataPtr->leftJoints[0]);
  if (!leftPos && _ecm.HasEntity(this->dataPtr->leftJoints[0]))
  {
    _ecm.CreateComponent(this->dataPtr->leftJoints[0],
      components::JointPosition());
  }

  auto rightPos = _ecm.Component<components::JointPosition>(
    this->dataPtr->rightJoints[0]);
  if (!rightPos && _ecm.HasEntity(this->dataPtr->rightJoints[0]))
  {
    _ecm.CreateComponent(this->dataPtr->rightJoints[0],
      components::JointPosition());
  }
}

//////////////////////////////////////////////////
void DiffDrive_second_order::PostUpdate(const UpdateInfo& _info,
  const EntityComponentManager& _ecm)
{
  IGN_PROFILE("DiffDrive::PostUpdate");
  // Nothing left to do if paused.
  if (_info.paused)
    return;

  this->dataPtr->UpdateVelocity(_info, _ecm);
  this->dataPtr->UpdateOdometry(_info, _ecm);
}

//////////////////////////////////////////////////
double DiffDrive_second_order_Private::ApplySecondOrderFilter(double _input,
  SecondOrderFilterState& _filter_state, double _dt)
{
  std::vector<double> b_coef = { b_0_, b_1_, b_2_ };
  std::vector<double> a_coef = { a_0_, a_1_, a_2_ };

  // Update u and y values (not 'x'_0 values)
  for (int i = _filter_state.order; i > 0; i--) {
    _filter_state.u_hist[i] = _filter_state.u_hist[i - 1];
    _filter_state.y_hist[i] = _filter_state.y_hist[i - 1];
  }

  // Update u_0 value with _input
  _filter_state.u_hist[0] = _input;

  // Compute output y_hist[0]
  _filter_state.y_hist[0] = 0.0000;

  for (int i = 0; i <= _filter_state.order; i++) {
    _filter_state.y_hist[0] += b_coef[i] * _filter_state.u_hist[i];
  }

  for (int i = 1; i <= _filter_state.order; i++) {
    _filter_state.y_hist[0] -= a_coef[i] * _filter_state.y_hist[i];
  }

  return _filter_state.y_hist[0];
}

//////////////////////////////////////////////////
double DiffDrive_second_order_Private::ApplyPureDelay(double _input,
  std::deque<double>& _buffer, std::size_t _delaySamples)
{
  _buffer.push_back(_input);

  if (_buffer.size() <= _delaySamples) {
    return 0.0;
  }

  double delayedValue = _buffer.front();
  _buffer.pop_front();
  return delayedValue;
}

//////////////////////////////////////////////////
void DiffDrive_second_order_Private::UpdateOdometry(const UpdateInfo& _info,
  const EntityComponentManager& _ecm)
{
  IGN_PROFILE("DiffDrive::UpdateOdometry");
  // Initialize, if not already initialized.
  if (!this->odom.Initialized())
  {
    this->odom.Init(std::chrono::steady_clock::time_point(_info.simTime));
    return;
  }

  if (this->leftJoints.empty() || this->rightJoints.empty())
    return;

  // Get the first joint positions for the left and right side.
  auto leftPos = _ecm.Component<components::JointPosition>(this->leftJoints[0]);
  auto rightPos = _ecm.Component<components::JointPosition>(
    this->rightJoints[0]);

  // Abort if the joints were not found or just created.
  if (!leftPos || !rightPos || leftPos->Data().empty() ||
    rightPos->Data().empty())
  {
    return;
  }

  this->odom.Update(leftPos->Data()[0], rightPos->Data()[0],
    std::chrono::steady_clock::time_point(_info.simTime));

  // Throttle publishing
  auto diff = _info.simTime - this->lastOdomPubTime;
  if (diff > std::chrono::steady_clock::duration::zero() &&
    diff < this->odomPubPeriod)
  {
    return;
  }
  this->lastOdomPubTime = _info.simTime;

  // Construct the odometry message and publish it.
  msgs::Odometry msg;
  msg.mutable_pose()->mutable_position()->set_x(this->odom.X());
  msg.mutable_pose()->mutable_position()->set_y(this->odom.Y());

  math::Quaterniond orientation(0, 0, *this->odom.Heading());
  msgs::Set(msg.mutable_pose()->mutable_orientation(), orientation);

  msg.mutable_twist()->mutable_linear()->set_x(this->odom.LinearVelocity());
  msg.mutable_twist()->mutable_angular()->set_z(*this->odom.AngularVelocity());

  // Set the time stamp in the header
  msg.mutable_header()->mutable_stamp()->CopyFrom(
    convert<msgs::Time>(_info.simTime));

  // Set the frame id.
  auto frame = msg.mutable_header()->add_data();
  frame->set_key("frame_id");
  if (this->sdfFrameId.empty())
  {
    frame->add_value(this->model.Name(_ecm) + "/odom");
  }
  else
  {
    frame->add_value(this->sdfFrameId);
  }

  std::optional<std::string> linkName = this->canonicalLink.Name(_ecm);
  if (this->sdfChildFrameId.empty())
  {
    if (linkName)
    {
      auto childFrame = msg.mutable_header()->add_data();
      childFrame->set_key("child_frame_id");
      childFrame->add_value(this->model.Name(_ecm) + "/" + *linkName);
    }
  }
  else
  {
    auto childFrame = msg.mutable_header()->add_data();
    childFrame->set_key("child_frame_id");
    childFrame->add_value(this->sdfChildFrameId);
  }

  // Construct the Pose_V/tf message and publish it.
  msgs::Pose_V tfMsg;
  msgs::Pose* tfMsgPose = tfMsg.add_pose();
  tfMsgPose->mutable_header()->CopyFrom(*msg.mutable_header());
  tfMsgPose->mutable_position()->CopyFrom(msg.mutable_pose()->position());
  tfMsgPose->mutable_orientation()->CopyFrom(msg.mutable_pose()->orientation());

  // Publish the messages
  this->odomPub.Publish(msg);
  this->tfPub.Publish(tfMsg);
}

//////////////////////////////////////////////////
void DiffDrive_second_order_Private::UpdateVelocity(const UpdateInfo& _info,
  const EntityComponentManager&/*_ecm*/)
{
  IGN_PROFILE("DiffDrive::UpdateVelocity");

  double linVel;
  double angVel;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    linVel = this->targetVel.linear().x();
    angVel = this->targetVel.angular().z();
  }

  // Apply delay if enabled
  if (this->enablePureDelay)
  {
    // Delay defined for constant 1 ms sample time
    const double dtSafe = 1e-3;
    const std::size_t delaySamples = static_cast<std::size_t>(std::round(this->pureDelaySeconds / dtSafe));

    linVel = this->ApplyPureDelay(linVel, this->linearDelayBuffer, delaySamples);
    angVel = this->ApplyPureDelay(angVel, this->angularDelayBuffer, delaySamples);
  }


  // Apply second order filter if enabled
  if (this->enableSecondOrderFilter)
  {
    double integration_dt = std::chrono::duration<double>(_info.dt).count();

    linVel = this->ApplySecondOrderFilter(linVel, this->linearFilterState, integration_dt);
    angVel = this->ApplySecondOrderFilter(angVel, this->angularFilterState, integration_dt);

  }


  // Limit the target velocity if needed.
  this->limiterLin->Limit(
    linVel, this->last0Cmd.lin, this->last1Cmd.lin, _info.dt);
  this->limiterAng->Limit(
    angVel, this->last0Cmd.ang, this->last1Cmd.ang, _info.dt);

  // Update history of commands.
  this->last1Cmd = last0Cmd;
  this->last0Cmd.lin = linVel;
  this->last0Cmd.ang = angVel;

  // Convert the target velocities to joint velocities.
  this->rightJointSpeed =
    (linVel + angVel * this->wheelSeparation / 2.0) / this->wheelRadius;
  this->leftJointSpeed =
    (linVel - angVel * this->wheelSeparation / 2.0) / this->wheelRadius;
}

//////////////////////////////////////////////////
void DiffDrive_second_order_Private::OnCmdVel(const msgs::Twist& _msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  if (this->enabled)
  {
    this->targetVel = _msg;
  }
}

//////////////////////////////////////////////////
void DiffDrive_second_order_Private::OnEnable(const msgs::Boolean& _msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->enabled = _msg.data();
  if (!this->enabled)
  {
    math::Vector3d zeroVector{ 0, 0, 0 };
    msgs::Set(this->targetVel.mutable_linear(), zeroVector);
    msgs::Set(this->targetVel.mutable_angular(), zeroVector);

    // Reset filter states when disabled
    if (this->enableSecondOrderFilter)
    {
      this->linearFilterState = SecondOrderFilterState();
      this->angularFilterState = SecondOrderFilterState();
    }
  }
}

IGNITION_ADD_PLUGIN(DiffDrive_second_order,
  ignition::gazebo::System,
  ignition::gazebo::ISystemConfigure,
  ignition::gazebo::ISystemPreUpdate,
  ignition::gazebo::ISystemPostUpdate)

  IGNITION_ADD_PLUGIN_ALIAS(DiffDrive_second_order, "ignition::gazebo::systems::DiffDrive_second_order")

