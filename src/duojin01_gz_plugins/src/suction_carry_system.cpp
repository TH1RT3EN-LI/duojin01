#include <atomic>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Types.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/PoseCmd.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace duojin01::gz_plugins
{

struct AttachProfile
{
  std::string modelPrefix;
  double pickupRadiusXY{0.018};
  double pickupWindowZ{0.010};
  double surfaceOffsetZ{0.015};
  double holdRecess{0.004};
};

class SuctionCarrySystem
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate
{
  public: void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager & /*_eventMgr*/) override
  {
    this->robotEntity = _entity;
    this->robotModel = gz::sim::Model(_entity);

    if (!this->robotModel.Valid(_ecm))
    {
      gzerr << "[SuctionCarrySystem] Plugin must be attached to a model." << std::endl;
      return;
    }

    if (_sdf->HasElement("suction_link"))
      this->suctionLinkName = _sdf->Get<std::string>("suction_link");
    if (_sdf->HasElement("suction_offset_xyz"))
      this->suctionOffsetPose.Pos() = _sdf->Get<gz::math::Vector3d>("suction_offset_xyz");
    if (_sdf->HasElement("suction_offset_rpy"))
    {
      const auto rpy = _sdf->Get<gz::math::Vector3d>("suction_offset_rpy");
      this->suctionOffsetPose.Rot() = gz::math::Quaterniond(rpy.X(), rpy.Y(), rpy.Z());
    }
    if (_sdf->HasElement("command_topic"))
      this->commandTopic = _sdf->Get<std::string>("command_topic");
    if (_sdf->HasElement("attachable_model_prefix"))
      this->primaryProfile.modelPrefix = _sdf->Get<std::string>("attachable_model_prefix");
    if (_sdf->HasElement("pickup_radius_xy"))
      this->primaryProfile.pickupRadiusXY = _sdf->Get<double>("pickup_radius_xy");
    if (_sdf->HasElement("pickup_window_z"))
      this->primaryProfile.pickupWindowZ = _sdf->Get<double>("pickup_window_z");
    if (_sdf->HasElement("max_relative_speed"))
      this->maxRelativeSpeed = _sdf->Get<double>("max_relative_speed");
    if (_sdf->HasElement("release_clearance"))
      this->releaseClearance = _sdf->Get<double>("release_clearance");
    if (_sdf->HasElement("suction_collision_positive_extent_z"))
      this->suctionCollisionPositiveExtentZ =
          std::max(0.0, _sdf->Get<double>("suction_collision_positive_extent_z"));
    if (_sdf->HasElement("suction_collision_negative_extent_z"))
      this->suctionCollisionNegativeExtentZ =
          std::max(0.0, _sdf->Get<double>("suction_collision_negative_extent_z"));
    if (_sdf->HasElement("hold_recess"))
      this->primaryProfile.holdRecess = _sdf->Get<double>("hold_recess");
    if (_sdf->HasElement("surface_offset_z"))
      this->primaryProfile.surfaceOffsetZ = _sdf->Get<double>("surface_offset_z");
    if (_sdf->HasElement("one_cube_only"))
      this->oneCubeOnly = _sdf->Get<bool>("one_cube_only");
    if (_sdf->HasElement("bag_model_prefix"))
      this->bagProfile.modelPrefix = _sdf->Get<std::string>("bag_model_prefix");
    if (_sdf->HasElement("bag_pickup_radius_xy"))
      this->bagProfile.pickupRadiusXY = _sdf->Get<double>("bag_pickup_radius_xy");
    if (_sdf->HasElement("bag_pickup_window_z"))
      this->bagProfile.pickupWindowZ = _sdf->Get<double>("bag_pickup_window_z");
    if (_sdf->HasElement("bag_surface_offset_z"))
      this->bagProfile.surfaceOffsetZ = _sdf->Get<double>("bag_surface_offset_z");
    if (_sdf->HasElement("bag_hold_recess"))
      this->bagProfile.holdRecess = _sdf->Get<double>("bag_hold_recess");

    this->suctionLinkEntity = this->robotModel.LinkByName(_ecm, this->suctionLinkName);
    if (this->suctionLinkEntity == gz::sim::kNullEntity)
    {
      gzerr << "[SuctionCarrySystem] Failed to find suction link ["
            << this->suctionLinkName << "] on model ["
            << this->robotModel.Name(_ecm) << "]." << std::endl;
      return;
    }

    gz::sim::Link(this->suctionLinkEntity).EnableVelocityChecks(_ecm);

    if (!this->oneCubeOnly)
    {
      gzwarn << "[SuctionCarrySystem] one_cube_only=false requested, but this implementation "
             << "still carries a single cube at a time." << std::endl;
    }

    if (!this->transportNode.Subscribe(
            this->commandTopic, &SuctionCarrySystem::OnCommand, this))
    {
      gzerr << "[SuctionCarrySystem] Failed to subscribe to command topic ["
            << this->commandTopic << "]." << std::endl;
      return;
    }

    gzmsg << "[SuctionCarrySystem] configured with base link ["
          << this->suctionLinkName << "], offset pose="
          << this->suctionOffsetPose << ", command topic ["
          << this->commandTopic << "], release_clearance="
          << this->releaseClearance << ", suction collision extents z=[-"
          << this->suctionCollisionNegativeExtentZ << ", +"
          << this->suctionCollisionPositiveExtentZ << "]." << std::endl;
    this->configured = true;
  }

  public: void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (!this->configured || _info.paused)
      return;

    this->FlushPendingCommandCleanup(_ecm);

    if (this->suctionLinkEntity == gz::sim::kNullEntity ||
        !_ecm.HasEntity(this->suctionLinkEntity))
    {
      this->suctionLinkEntity = this->robotModel.LinkByName(_ecm, this->suctionLinkName);
      if (this->suctionLinkEntity == gz::sim::kNullEntity)
        return;
    }

    gz::sim::Link suctionLink(this->suctionLinkEntity);
    suctionLink.EnableVelocityChecks(_ecm);

    const auto suctionLinkPose = suctionLink.WorldPose(_ecm);
    if (!suctionLinkPose.has_value())
      return;
    const gz::math::Pose3d suctionPose = (*suctionLinkPose) * this->suctionOffsetPose;

    gz::math::Vector3d suctionLinearVelocity = gz::math::Vector3d::Zero;
    if (const auto linearVelocity = suctionLink.WorldLinearVelocity(_ecm);
        linearVelocity.has_value())
    {
      suctionLinearVelocity = *linearVelocity;
    }

    gz::math::Vector3d suctionAngularVelocity = gz::math::Vector3d::Zero;
    if (const auto angularVelocity = suctionLink.WorldAngularVelocity(_ecm);
        angularVelocity.has_value())
    {
      suctionAngularVelocity = *angularVelocity;
    }

    if (!this->suctionEnabled.load(std::memory_order_relaxed))
    {
      if (this->attachedCubeEntity != gz::sim::kNullEntity)
        this->ReleaseCube(
            *suctionLinkPose,
            suctionLinearVelocity,
            suctionAngularVelocity,
            _ecm);
      return;
    }

    if (this->attachedCubeEntity == gz::sim::kNullEntity)
      this->TryAcquireCube(suctionPose, suctionLinearVelocity, _ecm);

    if (this->attachedCubeEntity != gz::sim::kNullEntity)
      this->HoldAttachedCube(suctionPose, _ecm);
  }

  private: void OnCommand(const gz::msgs::Boolean &_msg)
  {
    this->suctionEnabled.store(_msg.data(), std::memory_order_relaxed);
  }

  private: void TryAcquireCube(
      const gz::math::Pose3d &_suctionPose,
      const gz::math::Vector3d &_suctionVelocity,
      gz::sim::EntityComponentManager &_ecm)
  {
    gz::sim::Entity bestModel = gz::sim::kNullEntity;
    gz::sim::Entity bestCanonicalLink = gz::sim::kNullEntity;
    gz::math::Quaterniond bestRotation = gz::math::Quaterniond::Identity;
    std::string bestModelName;
    double bestScore = std::numeric_limits<double>::infinity();
    double bestNormalSign = 1.0;

    _ecm.Each<gz::sim::components::Model, gz::sim::components::Name>(
        [&](const gz::sim::Entity &_entity,
            const gz::sim::components::Model * /*_model*/,
            const gz::sim::components::Name *_name) -> bool
        {
          if (_entity == this->robotEntity || _name == nullptr)
            return true;

          const std::string &modelName = _name->Data();
          const AttachProfile *profile = this->ProfileForModel(modelName);
          if (profile == nullptr)
            return true;

          gz::sim::Model cubeModel(_entity);
          if (!cubeModel.Valid(_ecm) || cubeModel.Static(_ecm))
            return true;

          const gz::sim::Entity canonicalLinkEntity = cubeModel.CanonicalLink(_ecm);
          if (canonicalLinkEntity == gz::sim::kNullEntity)
            return true;

          gz::sim::Link cubeLink(canonicalLinkEntity);
          cubeLink.EnableVelocityChecks(_ecm);

          const auto cubePose = cubeLink.WorldPose(_ecm);
          const auto cubeVelocity = cubeLink.WorldLinearVelocity(_ecm);
          if (!cubePose.has_value() || !cubeVelocity.has_value())
            return true;

          const gz::math::Vector3d localCenter =
              _suctionPose.Rot().RotateVectorReverse(cubePose->Pos() - _suctionPose.Pos());
          const double lateralDistance = std::hypot(localCenter.X(), localCenter.Y());
          if (lateralDistance > profile->pickupRadiusXY)
            return true;

          const double faceDistance =
              std::abs(std::abs(localCenter.Z()) - profile->surfaceOffsetZ);
          if (faceDistance > profile->pickupWindowZ)
            return true;

          const double relativeSpeed = (*cubeVelocity - _suctionVelocity).Length();
          if (relativeSpeed > this->maxRelativeSpeed)
            return true;

          const double score = lateralDistance + faceDistance;
          if (score < bestScore)
          {
            bestScore = score;
            bestModel = _entity;
            bestCanonicalLink = canonicalLinkEntity;
            bestRotation = cubePose->Rot();
            bestModelName = modelName;
            bestNormalSign = localCenter.Z() >= 0.0 ? 1.0 : -1.0;
            this->pendingSurfaceOffsetZ = profile->surfaceOffsetZ;
            this->pendingHoldRecess = profile->holdRecess;
          }
          return true;
        });

    if (bestModel == gz::sim::kNullEntity)
      return;

    this->attachedCubeEntity = bestModel;
    this->attachedCanonicalLinkEntity = bestCanonicalLink;
    this->attachedCubeRotation = bestRotation;
    this->attachedModelName = bestModelName;
    this->attachedNormalSign = bestNormalSign;
    this->attachedSurfaceOffsetZ = this->pendingSurfaceOffsetZ;
    this->attachedHoldRecess = this->pendingHoldRecess;

    gzmsg << "[SuctionCarrySystem] attached model [" << this->attachedModelName
          << "], normal_sign=" << this->attachedNormalSign
          << ", surface_offset_z=" << this->attachedSurfaceOffsetZ
          << ", hold_recess=" << this->attachedHoldRecess
          << ", active_collision_extent="
          << this->ActiveSuctionCollisionExtent(this->attachedNormalSign)
          << "." << std::endl;

    this->HoldAttachedCube(_suctionPose, _ecm);
  }

  private: void HoldAttachedCube(
      const gz::math::Pose3d &_suctionPose,
      gz::sim::EntityComponentManager &_ecm)
  {
    if (this->attachedCubeEntity == gz::sim::kNullEntity ||
        !_ecm.HasEntity(this->attachedCubeEntity))
    {
      this->ClearAttachmentState();
      return;
    }

    gz::sim::Model cubeModel(this->attachedCubeEntity);
    if (!cubeModel.Valid(_ecm))
    {
      this->ClearAttachmentState();
      return;
    }

    if (this->attachedCanonicalLinkEntity == gz::sim::kNullEntity ||
        !_ecm.HasEntity(this->attachedCanonicalLinkEntity))
    {
      this->attachedCanonicalLinkEntity = cubeModel.CanonicalLink(_ecm);
      if (this->attachedCanonicalLinkEntity == gz::sim::kNullEntity)
      {
        this->ClearAttachmentState();
        return;
      }
    }

    gz::math::Pose3d desiredPose = _suctionPose;
    const gz::math::Vector3d localHoldOffset(
        0.0, 0.0, this->attachedNormalSign * (this->attachedSurfaceOffsetZ - this->attachedHoldRecess));
    desiredPose.Pos() += _suctionPose.Rot().RotateVector(localHoldOffset);
    desiredPose.Rot() = this->attachedCubeRotation;

    cubeModel.SetWorldPoseCmd(_ecm, desiredPose);
    this->SetLinkVelocity(
        this->attachedCanonicalLinkEntity,
        gz::math::Vector3d::Zero,
        gz::math::Vector3d::Zero,
        _ecm);
  }

  private: void ReleaseCube(
      const gz::math::Pose3d &_suctionLinkPose,
      const gz::math::Vector3d &_suctionLinkLinearVelocity,
      const gz::math::Vector3d &_suctionLinkAngularVelocity,
      gz::sim::EntityComponentManager &_ecm)
  {
    if (this->attachedCubeEntity == gz::sim::kNullEntity ||
        !_ecm.HasEntity(this->attachedCubeEntity))
    {
      this->ClearAttachmentState();
      return;
    }

    gz::sim::Model cubeModel(this->attachedCubeEntity);
    if (!cubeModel.Valid(_ecm))
    {
      this->ClearAttachmentState();
      return;
    }

    if (this->attachedCanonicalLinkEntity == gz::sim::kNullEntity ||
        !_ecm.HasEntity(this->attachedCanonicalLinkEntity))
    {
      this->attachedCanonicalLinkEntity = cubeModel.CanonicalLink(_ecm);
      if (this->attachedCanonicalLinkEntity == gz::sim::kNullEntity)
      {
        this->ClearAttachmentState();
        return;
      }
    }

    gz::math::Pose3d releasePose = _suctionLinkPose * this->suctionOffsetPose;
    const double activeCollisionExtent =
        this->ActiveSuctionCollisionExtent(this->attachedNormalSign);
    const double releaseCenterDistance =
        this->attachedSurfaceOffsetZ + this->releaseClearance + activeCollisionExtent;
    const double releaseFaceDistance =
        std::max(0.0, releaseCenterDistance - this->attachedSurfaceOffsetZ);
    const double residualOverlapEstimate =
        std::max(0.0, activeCollisionExtent - releaseFaceDistance);

    const gz::math::Vector3d releaseOffset(
        0.0,
        0.0,
        this->attachedNormalSign * releaseCenterDistance);
    releasePose.Pos() += releasePose.Rot().RotateVector(releaseOffset);
    releasePose.Rot() = this->attachedCubeRotation;

    const gz::math::Vector3d releaseLinearVelocity =
        _suctionLinkLinearVelocity +
        _suctionLinkAngularVelocity.Cross(
            releasePose.Pos() - _suctionLinkPose.Pos());

    cubeModel.SetWorldPoseCmd(_ecm, releasePose);
    this->SetLinkVelocity(
        this->attachedCanonicalLinkEntity,
        releaseLinearVelocity,
        _suctionLinkAngularVelocity,
        _ecm);
    this->ScheduleCommandCleanup(
        this->attachedCubeEntity,
        this->attachedCanonicalLinkEntity);

    gzmsg << "[SuctionCarrySystem] released model [" << this->attachedModelName
          << "], center_distance=" << releaseCenterDistance
          << ", face_distance=" << releaseFaceDistance
          << ", active_collision_extent=" << activeCollisionExtent
          << ", residual_overlap_est=" << residualOverlapEstimate
          << ", release_pose=" << releasePose
          << ", linear_vel=" << releaseLinearVelocity
          << ", angular_vel=" << _suctionLinkAngularVelocity
          << "." << std::endl;
    this->ClearAttachmentState();
  }

  private: void ScheduleCommandCleanup(
      gz::sim::Entity _modelEntity,
      gz::sim::Entity _linkEntity)
  {
    this->pendingCleanupModelEntity = _modelEntity;
    this->pendingCleanupLinkEntity = _linkEntity;
    this->pendingCleanupActive = true;
  }

  private: void FlushPendingCommandCleanup(
      gz::sim::EntityComponentManager &_ecm)
  {
    if (!this->pendingCleanupActive)
      return;

    if (this->pendingCleanupModelEntity != gz::sim::kNullEntity &&
        _ecm.HasEntity(this->pendingCleanupModelEntity))
    {
      _ecm.RemoveComponent<gz::sim::components::WorldPoseCmd>(
          this->pendingCleanupModelEntity);
    }

    if (this->pendingCleanupLinkEntity != gz::sim::kNullEntity &&
        _ecm.HasEntity(this->pendingCleanupLinkEntity))
    {
      _ecm.RemoveComponent<gz::sim::components::LinearVelocityCmd>(
          this->pendingCleanupLinkEntity);
      _ecm.RemoveComponent<gz::sim::components::WorldLinearVelocityCmd>(
          this->pendingCleanupLinkEntity);
      _ecm.RemoveComponent<gz::sim::components::AngularVelocityCmd>(
          this->pendingCleanupLinkEntity);
      _ecm.RemoveComponent<gz::sim::components::WorldAngularVelocityCmd>(
          this->pendingCleanupLinkEntity);
    }

    gzmsg << "[SuctionCarrySystem] cleared pending pose/velocity command components for model entity ["
          << this->pendingCleanupModelEntity << "], link entity ["
          << this->pendingCleanupLinkEntity << "]." << std::endl;

    this->pendingCleanupModelEntity = gz::sim::kNullEntity;
    this->pendingCleanupLinkEntity = gz::sim::kNullEntity;
    this->pendingCleanupActive = false;
  }

  private: void SetLinkVelocity(
      gz::sim::Entity _linkEntity,
      const gz::math::Vector3d &_linearVelocity,
      const gz::math::Vector3d &_angularVelocity,
      gz::sim::EntityComponentManager &_ecm)
  {
    if (_linkEntity == gz::sim::kNullEntity || !_ecm.HasEntity(_linkEntity))
      return;

    gz::sim::Link link(_linkEntity);
    link.SetLinearVelocity(_ecm, _linearVelocity);
    link.SetAngularVelocity(_ecm, _angularVelocity);
  }

  private: void ClearAttachmentState()
  {
    this->attachedCubeEntity = gz::sim::kNullEntity;
    this->attachedCanonicalLinkEntity = gz::sim::kNullEntity;
    this->attachedCubeRotation = gz::math::Quaterniond::Identity;
    this->attachedModelName.clear();
    this->attachedNormalSign = 1.0;
    this->attachedSurfaceOffsetZ = this->primaryProfile.surfaceOffsetZ;
    this->attachedHoldRecess = this->primaryProfile.holdRecess;
    this->pendingSurfaceOffsetZ = this->primaryProfile.surfaceOffsetZ;
    this->pendingHoldRecess = this->primaryProfile.holdRecess;
  }

  private: double ActiveSuctionCollisionExtent(double _normalSign) const
  {
    return _normalSign >= 0.0
        ? this->suctionCollisionPositiveExtentZ
        : this->suctionCollisionNegativeExtentZ;
  }

  private: const AttachProfile *ProfileForModel(const std::string &_modelName) const
  {
    if (!this->primaryProfile.modelPrefix.empty() &&
        _modelName.rfind(this->primaryProfile.modelPrefix, 0) == 0)
    {
      return &this->primaryProfile;
    }

    if (!this->bagProfile.modelPrefix.empty() &&
        _modelName.rfind(this->bagProfile.modelPrefix, 0) == 0)
    {
      return &this->bagProfile;
    }

    return nullptr;
  }

  private: bool configured{false};
  private: std::atomic_bool suctionEnabled{false};
  private: bool oneCubeOnly{true};

  private: std::string suctionLinkName{"arm_suction_link"};
  private: gz::math::Pose3d suctionOffsetPose{
      gz::math::Vector3d::Zero,
      gz::math::Quaterniond::Identity};
  private: std::string commandTopic{"/duojin01/suction_enable"};
  private: AttachProfile primaryProfile{
      "tag_cube_", 0.018, 0.010, 0.015, 0.004};
  private: AttachProfile bagProfile{
      "", 0.070, 0.020, 0.090, 0.010};

  private: double maxRelativeSpeed{0.10};
  private: double releaseClearance{0.001};
  private: double suctionCollisionPositiveExtentZ{0.0};
  private: double suctionCollisionNegativeExtentZ{0.0};
  private: double attachedNormalSign{1.0};
  private: double attachedSurfaceOffsetZ{0.015};
  private: double attachedHoldRecess{0.004};
  private: double pendingSurfaceOffsetZ{0.015};
  private: double pendingHoldRecess{0.004};

  private: gz::sim::Entity robotEntity{gz::sim::kNullEntity};
  private: gz::sim::Entity suctionLinkEntity{gz::sim::kNullEntity};
  private: gz::sim::Entity attachedCubeEntity{gz::sim::kNullEntity};
  private: gz::sim::Entity attachedCanonicalLinkEntity{gz::sim::kNullEntity};
  private: gz::sim::Entity pendingCleanupModelEntity{gz::sim::kNullEntity};
  private: gz::sim::Entity pendingCleanupLinkEntity{gz::sim::kNullEntity};
  private: bool pendingCleanupActive{false};

  private: gz::math::Quaterniond attachedCubeRotation{
      gz::math::Quaterniond::Identity};
  private: std::string attachedModelName;

  private: gz::sim::Model robotModel;
  private: gz::transport::Node transportNode;
};

}  // namespace duojin01::gz_plugins

GZ_ADD_PLUGIN(
    duojin01::gz_plugins::SuctionCarrySystem,
    gz::sim::System,
    gz::sim::ISystemConfigure,
    gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    duojin01::gz_plugins::SuctionCarrySystem,
    "duojin01::gz_plugins::SuctionCarrySystem")
