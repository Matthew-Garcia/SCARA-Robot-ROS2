#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Pose3.hh>

#include <functional>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace gazebo
{
class ScaraGraspWorldPlugin final : public SystemPlugin
{
public:
  void Load(int, char **) override
  {
    target_names_ = {
      "pickup_cube", "pickup_sphere", "pickup_cylinder", "pickup_hex_prism",
      "hanoi_ring_large", "hanoi_ring_medium", "hanoi_ring_small",
      "conveyor_cube_red", "conveyor_cube_green", "conveyor_cube_blue"};
    world_created_connection_ = event::Events::ConnectWorldCreated(
      std::bind(&ScaraGraspWorldPlugin::OnWorldCreated, this, std::placeholders::_1));
    gzerr << "[scara_grasp] System plugin loaded.\n";
  }

private:
  void OnWorldCreated(const std::string & world_name)
  {
    world_ = physics::get_world(world_name);
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&ScaraGraspWorldPlugin::OnUpdate, this));
    gzerr << "[scara_grasp] Connected to world " << world_name << ".\n";
  }

  bool ResolveRobot()
  {
    if (robot_ && gripper_link_ && left_joint_ && right_joint_) return true;
    robot_ = world_->ModelByName(robot_name_);
    if (!robot_) return false;
    gripper_link_ = robot_->GetLink(gripper_link_name_);
    left_joint_ = robot_->GetJoint(left_joint_name_);
    right_joint_ = robot_->GetJoint(right_joint_name_);
    if (!gripper_link_ || !left_joint_ || !right_joint_) {
      robot_.reset();
      return false;
    }
    gzmsg << "[scara_grasp] Resolved robot gripper and finger joints.\n";
    return true;
  }

  void OnUpdate()
  {
    if (!ResolveRobot()) return;
    const double left = left_joint_->Position(0);
    const double right = right_joint_->Position(0);
    if (attached_joint_) {
      if (left >= open_threshold_ || right >= open_threshold_) {
        const auto name = attached_model_ ? attached_model_->GetName() : "object";
        attached_joint_->Detach();
        attached_joint_->Fini();
        attached_joint_.reset();
        attached_model_.reset();
        gzmsg << "[scara_grasp] Released " << name << ".\n";
      }
      return;
    }
    if (left > closed_threshold_ || right > closed_threshold_) return;
    physics::ModelPtr nearest;
    physics::LinkPtr nearest_link;
    double nearest_distance = std::numeric_limits<double>::max();
    const auto grip_position = gripper_link_->WorldPose().Pos();
    for (const auto & name : target_names_) {
      auto candidate = world_->ModelByName(name);
      if (!candidate) continue;
      auto link = candidate->GetLink("body");
      if (!link) continue;
      const double distance = grip_position.Distance(link->WorldPose().Pos());
      if (distance < nearest_distance) {
        nearest = candidate;
        nearest_link = link;
        nearest_distance = distance;
      }
    }
    if (!nearest || nearest_distance > attach_distance_) return;
    attached_joint_ = world_->Physics()->CreateJoint("fixed", robot_);
    attached_joint_->Load(gripper_link_, nearest_link, ignition::math::Pose3d::Zero);
    attached_joint_->Init();
    attached_model_ = nearest;
    gzmsg << "[scara_grasp] Attached " << nearest->GetName() << " at "
          << nearest_distance << " m.\n";
  }

  physics::WorldPtr world_;
  physics::ModelPtr robot_;
  physics::LinkPtr gripper_link_;
  physics::JointPtr left_joint_;
  physics::JointPtr right_joint_;
  physics::JointPtr attached_joint_;
  physics::ModelPtr attached_model_;
  event::ConnectionPtr world_created_connection_;
  event::ConnectionPtr update_connection_;
  std::vector<std::string> target_names_;
  std::string robot_name_, gripper_link_name_, left_joint_name_, right_joint_name_;
  double attach_distance_{0.090}, closed_threshold_{0.006}, open_threshold_{0.016};
};

GZ_REGISTER_SYSTEM_PLUGIN(ScaraGraspWorldPlugin)
}  // namespace gazebo
