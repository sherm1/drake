#include "drake/multibody/tree/position_kinematics_cache.h"

#include "drake/multibody/tree/frame_body_pose_cache.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
void PositionKinematicsCache<T>::PrecomputeWorldComposite(
    const SpanningForest& forest,
    const FrameBodyPoseCache<T>& frame_body_pose_cache) {
  const SpanningForest::Mobod& world_mobod = forest.mobods(MobodIndex(0));
  const std::vector<LinkOrdinal>& world_followers =
      world_mobod.follower_link_ordinals();
  for (size_t i = 1; i < world_followers.size(); ++i) {
    const LinkOrdinal link_ordinal = world_followers[i];
    const math::RigidTransform<T>& X_WL =
        frame_body_pose_cache.get_X_BL(link_ordinal);  // B(=W) to link L
    SetX_WL(link_ordinal, X_WL);
  }
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::PositionKinematicsCache);
