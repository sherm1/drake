#pragma once

#include <memory>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/math/spatial_algebra.h"
#include "drake/multibody/topology/forest.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {
namespace internal {

/* This class is one of the cache entries in the Context. It holds the
kinematics results of computations that depend not only on the generalized
positions q and velocities v of the system, but also on their time derivatives.

- A_WB:   Spatial acceleration of mobod B in W for every Mobod. Frame B is
          the same as frame L₀ of a mobod's active link.
- A_WL:   Spatial acceleration of link L in W for every link. Indexed by
          LinkOrdinal. Same as A_WB if L is the active link of mobod B.
- vdot:   The system generalized accelerations, that is, the time derivative
          dv/dt of the system generalized velocities v.

@tparam_default_scalar */
template <typename T>
class AccelerationKinematicsCache {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(AccelerationKinematicsCache);

  // Constructs an acceleration kinematics cache entry for the given
  // SpanningForest.
  // In Release builds specific entries are left uninitialized resulting in a
  // zero cost operation. However in Debug builds those entries are set to NaN
  // so that operations using this uninitialized cache entry fail fast, easing
  // bug detection.
  explicit AccelerationKinematicsCache(const internal::SpanningForest& forest) {
    Allocate(forest);
    InitializeToNaN();
    // World's acceleration is always zero.
    A_WB_pool_[MobodIndex(0)].SetZero();
    A_WL_pool_[LinkOrdinal(0)].SetZero();
  }

  // For the body B associated with mobilized body `mobod_index`, returns A_WB,
  // body B's spatial acceleration in the world frame W.
  // This method aborts in Debug builds if `mobod_index` does not
  // correspond to a valid BodyNode in the MultibodyTree.
  // @param[in] mobod_index The unique index for the computational
  //                        BodyNode object associated with body B.
  // @retval A_WB_W body B's spatial acceleration in the world frame W,
  // expressed in W (for point Bo, the body's origin).
  const SpatialAcceleration<T>& get_A_WB(MobodIndex mobod_index) const {
    DRAKE_ASSERT(0 <= mobod_index && mobod_index < get_num_mobods());
    return A_WB_pool_[mobod_index];
  }

  // Mutable version of get_A_WB().
  SpatialAcceleration<T>& get_mutable_A_WB(MobodIndex mobod_index) {
    DRAKE_ASSERT(0 <= mobod_index && mobod_index < get_num_mobods());
    return A_WB_pool_[mobod_index];
  }

  // Returns a const reference to the pool of body accelerations.
  // The pool is returned as a `std::vector` of SpatialAcceleration objects
  // ordered by MobodIndex.
  // Most users should not need to call this method.
  const std::vector<SpatialAcceleration<T>>& get_A_WB_pool() const {
    return A_WB_pool_;
  }

  // Mutable version of get_A_WB_pool().
  std::vector<SpatialAcceleration<T>>& get_mutable_A_WB_pool() {
    return A_WB_pool_;
  }

  // Returns a constant reference to the generalized accelerations `vdot` for
  // the entire model.
  const VectorX<T>& get_vdot() const { return vdot_; }

  // Mutable version of get_vdot().
  VectorX<T>& get_mutable_vdot() { return vdot_; }

  // Set all acceleration fields to zero. This must be done explicitly; on
  // construction the fields are all set to NaN.
  void SetToZero() {
    for (SpatialAcceleration<T>& acc : A_WB_pool_) {
      acc.SetZero();
    }
    for (SpatialAcceleration<T>& acc : A_WL_pool_) {
      acc.SetZero();
    }
    vdot_.setZero();
  }

 private:
  int get_num_mobods() const { return ssize(A_WB_pool_); }
  int get_num_links() const { return ssize(A_WL_pool_); }

  // Allocates resources for this acceleration kinematics cache.
  void Allocate(const internal::SpanningForest& forest) {
    A_WB_pool_.resize(forest.num_mobods());
    A_WL_pool_.resize(forest.num_links());
    vdot_.resize(forest.num_velocities());
  }

  // Initializes all pools to have NaN values to ease bug detection when entries
  // are accidentally left uninitialized.
  void InitializeToNaN() {
    for (SpatialAcceleration<T>& acc : A_WB_pool_) {
      acc.SetNaN();
    }
    for (SpatialAcceleration<T>& acc : A_WL_pool_) {
      acc.SetNaN();
    }
    for (T& vdot : vdot_) {
      vdot = NAN;
    }
  }

  std::vector<SpatialAcceleration<T>> A_WB_pool_;  // Indexed by MobodIndex.
  std::vector<SpatialAcceleration<T>> A_WL_pool_;  // Indexed by LinkOrdinal.
  VectorX<T> vdot_;                                // 0..nv-1
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::AccelerationKinematicsCache);
