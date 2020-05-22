/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <ignition/plugin/Register.hh>

#include <ignition/common/Profiler.hh>

#include <ignition/gazebo/components/Actor.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/Util.hh>

#include "FollowActor.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

/// \brief Private FollowActor data class.
class ignition::gazebo::systems::FollowActorPrivate
{
  /// \brief Entity for the actor.
  public: Entity actorEntity{kNullEntity};

  /// \brief Velocity of the actor
  public: double velocity{0.8};

  /// \brief Current target to follow
  public: Entity targetEntity{kNullEntity};

  /// \brief Minimum distance in meters to keep away from target.
  public: double minDistance{1.2};

  /// \brief Maximum distance in meters to keep away from target.
  public: double maxDistance{4};

  /// \brief Time scaling factor. Used to coordinate translational motion
  /// with the actor's walking animation.
  public: double animationFactor{5.1};

  /// \brief Time of the last update.
  public: std::chrono::steady_clock::duration lastUpdate{0};
};

//////////////////////////////////////////////////
FollowActor::FollowActor() :
  System(), dataPtr(std::make_unique<FollowActorPrivate>())
{
}

//////////////////////////////////////////////////
FollowActor::~FollowActor() = default;

//////////////////////////////////////////////////
void FollowActor::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->actorEntity = _entity;

  if (!_sdf->HasElement("velocity"))
    return;

  auto targetName = _sdf->Get<std::string>("target");
  this->dataPtr->targetEntity = _ecm.EntityByComponents(components::Name(
      targetName));
  if (kNullEntity == this->dataPtr->targetEntity)
  {
    ignerr << "Failed to find target entity [" << targetName << "]"
           << std::endl;
    return;
  }

  if (_sdf->HasElement("velocity"))
    this->dataPtr->velocity = _sdf->Get<double>("velocity");

  if (_sdf->HasElement("min_distance"))
    this->dataPtr->minDistance = _sdf->Get<double>("min_distance");

  if (_sdf->HasElement("max_distance"))
    this->dataPtr->maxDistance = _sdf->Get<double>("max_distance");

  if (_sdf->HasElement("animation_factor"))
    this->dataPtr->animationFactor = _sdf->Get<double>("animation_factor");

  // Having a trajectory pose prevents the actor from moving with the
  // SDF script
  auto trajPoseComp = _ecm.Component<components::TrajectoryPose>(_entity);
  if (nullptr == trajPoseComp)
  {
    _ecm.CreateComponent(_entity, components::TrajectoryPose(
        math::Pose3d::Zero));
  }

  // Set custom animation time from this plugin
  auto animTimeComp = _ecm.Component<components::AnimationTime>(_entity);
  if (nullptr == animTimeComp)
  {
    _ecm.CreateComponent(_entity, components::AnimationTime());
  }

  // We'll be setting the actor's X/Y pose with respect to the world. So we
  // zero the current values.
  auto poseComp = _ecm.Component<components::Pose>(_entity);
  if (nullptr == trajPoseComp)
  {
    _ecm.CreateComponent(_entity, components::Pose(
        math::Pose3d::Zero));
  }
  else
  {
    auto newPose = poseComp->Data();
    newPose.Pos().X(0);
    newPose.Pos().Y(0);
    *poseComp = components::Pose(newPose);
  }
}

//////////////////////////////////////////////////
void FollowActor::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  IGN_PROFILE("FollowActor::PreUpdate");

  if (_info.paused)
    return;

  // TODO(louise) Throttle this system

  // Time delta
  std::chrono::duration<double> dtDuration = _info.simTime -
      this->dataPtr->lastUpdate;
  double dt = dtDuration.count();

  this->dataPtr->lastUpdate = _info.simTime;

  // Is there a follow target?
  if (this->dataPtr->targetEntity == kNullEntity)
    return;

  // Current world pose
  auto trajPoseComp = _ecm.Component<components::TrajectoryPose>(
      this->dataPtr->actorEntity);
  auto actorPose = trajPoseComp->Data();
  auto initialPose = actorPose;

  // Current target
  auto targetPose = _ecm.Component<components::Pose>(
      this->dataPtr->targetEntity)->Data();

  // Direction to target
  auto dir = targetPose.Pos() - actorPose.Pos();
  dir.Z(0);

  // Stop if too close to target
  if (dir.Length() <= this->dataPtr->minDistance)
  {
    return;
  }

  // Stop following if too far from target
  if (dir.Length() > this->dataPtr->maxDistance)
  {
    ignmsg << "Target [" << this->dataPtr->targetEntity
           <<  "] too far, actor [" << this->dataPtr->actorEntity
           <<"] stopped following" << std::endl;
    this->dataPtr->targetEntity = kNullEntity;

    return;
  }

  dir.Normalize();

  // Towards target
  math::Angle yaw = atan2(dir.Y(), dir.X());
  yaw.Normalize();

  actorPose.Pos() += dir * this->dataPtr->velocity * dt;
  actorPose.Pos().Z(0);
  actorPose.Rot() = math::Quaterniond(0, 0, yaw.Radian());

  // Distance traveled is used to coordinate motion with the walking
  // animation
  double distanceTraveled = (actorPose.Pos() - initialPose.Pos()).Length();

  // Update actor root pose
  *trajPoseComp = components::TrajectoryPose(actorPose);
  _ecm.SetChanged(this->dataPtr->actorEntity,
      components::TrajectoryPose::typeId, ComponentState::OneTimeChange);

  // Update actor bone trajectories based on animation time
  auto animTimeComp = _ecm.Component<components::AnimationTime>(
      this->dataPtr->actorEntity);

  auto animTime = animTimeComp->Data() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(distanceTraveled *
    this->dataPtr->animationFactor));
  *animTimeComp = components::AnimationTime(animTime);
  _ecm.SetChanged(this->dataPtr->actorEntity,
      components::AnimationTime::typeId, ComponentState::OneTimeChange);
}

IGNITION_ADD_PLUGIN(FollowActor, System,
  FollowActor::ISystemConfigure,
  FollowActor::ISystemPreUpdate
)

IGNITION_ADD_PLUGIN_ALIAS(FollowActor, "ignition::gazebo::systems::FollowActor")
