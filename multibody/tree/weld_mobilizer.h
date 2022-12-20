#pragma once

#include <limits>
#include <memory>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/tree/frame.h"
#include "drake/multibody/tree/mobilizer_impl.h"
#include "drake/multibody/tree/multibody_tree_topology.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace internal {

// This mobilizer fixes the relative pose `X_FM` of an outboard frame M in an
// inboard frame F as if "welding" them together at this fixed relative pose.
// Therefore, this mobilizer has no associated state with it.
//
// @tparam_default_scalar
template <typename T>
class WeldMobilizer final : public MobilizerImpl<T, 0, 0, WeldMobilizer> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(WeldMobilizer)

  static constexpr bool kCanRotate = false;
  static constexpr bool kCanTranslate = false;
  static constexpr bool kIsFloating = false;
  static constexpr bool kHasQuaternion = false;

  // Constructor for a %WeldMobilizer between the `inboard_frame_F` and
  // `outboard_frame_M`.
  // @param[in] X_FM Pose of `outboard_frame_M` in the `inboard_frame_F`.
  WeldMobilizer(const Frame<T>& inboard_frame_F,
                const Frame<T>& outboard_frame_M,
                const math::RigidTransform<double>& X_FM) :
      MobilizerBase(inboard_frame_F, outboard_frame_M), X_FM_(X_FM) {}

  // @retval X_FM The pose of the outboard frame M in the inboard frame F.
  const math::RigidTransform<double>& get_X_FM() const { return X_FM_; }

  // Computes the across-mobilizer transform `X_FM`, which for this mobilizer
  // is independent of the state stored in `context`.
  math::RigidTransform<T> CalcX_FM(const Vector<T, 0>&) const {
    return get_X_FM().template cast<T>();
  }

  // @param[in] dummy generalized coordinates (there are none for a Weld)
  // @param[in,out] X_FM transform already set to our X_FM value
  void FillX_FM(const Vector<T, 0>&,
                math::RigidTransform<T>* X_FM) const {
    unused(X_FM);  // except in Debug
    DRAKE_ASSERT(X_FM->IsExactlyEqualTo(get_X_FM().template cast<T>()));
  }

  // Computes the across-mobilizer velocity `V_FM` which for this mobilizer is
  // always zero since the outboard frame M is fixed to the inboard frame F.
  SpatialVelocity<T> CalcAcrossMobilizerSpatialVelocity(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& v) const final;

  // Computes the across-mobilizer acceleration `A_FM` which for this mobilizer
  // is always zero since the outboard frame M is fixed to the inboard frame F.
  SpatialAcceleration<T> CalcAcrossMobilizerSpatialAcceleration(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& vdot) const final;

  // Since this mobilizer has no generalized velocities associated with it,
  // this override is a no-op.
  void ProjectSpatialForce(
      const systems::Context<T>& context,
      const SpatialForce<T>& F_Mo_F,
      Eigen::Ref<VectorX<T>> tau) const final;

  // This override is a no-op since this mobilizer has no generalized
  // velocities associated with it.
  void MapVelocityToQDot(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& v,
      EigenPtr<VectorX<T>> qdot) const final;

  // This override is a no-op since this mobilizer has no generalized
  // velocities associated with it.
  void MapQDotToVelocity(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& qdot,
      EigenPtr<VectorX<T>> v) const final;

 protected:
  void DoCalcNMatrix(const systems::Context<T>& context,
                     EigenPtr<MatrixX<T>> N) const final;

  void DoCalcNplusMatrix(
      const systems::Context<T>& context,
      EigenPtr<MatrixX<T>> Nplus) const final;

  std::unique_ptr<Mobilizer<double>> DoCloneToScalar(
      const MultibodyTree<double>& tree_clone) const final;

  std::unique_ptr<Mobilizer<AutoDiffXd>> DoCloneToScalar(
      const MultibodyTree<AutoDiffXd>& tree_clone) const final;

  std::unique_ptr<Mobilizer<symbolic::Expression>> DoCloneToScalar(
      const MultibodyTree<symbolic::Expression>& tree_clone) const final;

 private:
  using MobilizerBase = MobilizerImpl<T, 0, 0, WeldMobilizer>;
  using MobilizerBase::kNq;
  using MobilizerBase::kNv;

  // Helper method to make a clone templated on ToScalar.
  template <typename ToScalar>
  std::unique_ptr<Mobilizer<ToScalar>> TemplatedDoCloneToScalar(
      const MultibodyTree<ToScalar>& tree_clone) const;

  // Pose of the outboard frame M in the inboard frame F.
  math::RigidTransform<double> X_FM_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::WeldMobilizer)
