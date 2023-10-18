#include "drake/multibody/topology/link_joint_graph.h"  // NOLINT

namespace drake {
namespace multibody {
namespace internal {

void LinkJointGraph::Link::clear_model(int num_user_joints) {
  mobod_ = {};
  joint_ = {};
  primary_link_ = {};
  shadow_links_.clear();
  composite_link_index_ = {};

  auto remove_model_joints =
      [num_user_joints](std::vector<JointIndex>& joints) {
        while (!joints.empty() && joints.back() >= num_user_joints)
          joints.pop_back();
      };

  remove_model_joints(joints_as_parent_);
  remove_model_joints(joints_as_child_);
  remove_model_joints(joints_);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
