#pragma once

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
namespace rendering {

// TODO(david-german-tri, SeanCurtis-TRI): Evolve PoseAggregator into
// GeometrySystem after GeometryWorld becomes available.

// TODO(david-german-tri): Rename PoseAggregator to KinematicsAggregator, since
// it includes both poses and velocities now.

/// PoseAggregator is a multiplexer for heterogeneous sources of poses and the
/// velocities of those poses.
/// Supported sources are:
/// - A PoseVector input, which is a single pose {R, p}, and is vector-valued.
/// - A FrameVelocity input, which corresponds to a PoseVector input, and
///   contains a single velocity {ω, v}, and is vector-valued.
/// - A PoseBundle input, which is a collection of poses and velocities, and is
///   abstract-valued.
///
/// PoseAggregator is stateless.
///
/// The output is a flat PoseBundle that contains all the poses and velocities
/// from all the inputs. Unspecified velocities are zero.
/// By convention, each aggregated pose or velocity is in the same world frame
/// of reference.
///
/// The output poses are named in the form `<source>` or `<source>::<pose>`.
/// - For poses derived from a PoseVector input, <source> is the bundle name
///   provided at construction time, and "::<pose>" is omitted.
/// - For poses derived from a PoseBundle input, <source> is the bundle name
///   provided at construction time, and <pose> is the name found in the input
///   PoseBundle at output evaluation time. In any sane use case, the input
///   names will be stable during a simulation, but PoseAggregator is stateless
///   and therefore can't check whether this is actually true.
///
/// The output poses are also each assigned a model instance ID, which must be
/// an integer that is greater than or equal to zero. All poses with the same
/// model instance ID must have unique names. This enables PoseAggregator to
/// aggregate multiple instances of the same model.
/// - For poses derived from a PoseVector input, the instance ID is specified
///   when the input is declared.
/// - For poses derived from a PoseBundle input, the instance ID is obtained
///   directly from the PoseBundle.
///
/// In typical usage, Diagrams should contain just one PoseAggregator, and
/// every pose in the Diagram should appear as an input to it. Then,
/// Systems that need to ingest every pose in the universe, such as renderers
/// or sensor models, can simply depend on the output.
///
/// This class is explicitly instantiated for the following scalar types. No
/// other scalar types are supported.
/// - double
/// - AutoDiffXd
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///           Only double and AutoDiffXd are supported.
template <typename T>
class PoseAggregator : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PoseAggregator)

  PoseAggregator();
  ~PoseAggregator() override;

  /// Adds an input for a PoseVector. @p name must be unique for all inputs with
  /// the same @p model_instance_id.
  const InputPortDescriptor<T>& AddSingleInput(const std::string& name,
                                               int model_instance_id);

  /// Adds an input for a PoseVector, and a corresponding input for a
  /// FrameVelocity. @p name must be unique for all inputs with the same
  /// @p model_instance_id.
  ///
  /// @return A pair where the first element is the descriptor for the pose
  ///         input, and the second element is the descriptor for the velocity
  ///         input.
  std::pair<const InputPortDescriptor<T>&, const InputPortDescriptor<T>&>
  AddSinglePoseAndVelocityInput(const std::string& name, int model_instance_id);

  /// Adds an input for a PoseBundle containing @p num_poses poses.
  const InputPortDescriptor<T>& AddBundleInput(const std::string& bundle_name,
                                               int num_poses);

  /// Aggregates the input poses into the output PoseBundle, in the order
  /// the input ports were added. Aborts if any inputs have an unexpected
  /// dimension.
  void DoCalcOutput(const Context<T>& context,
                    SystemOutput<T>* output) const override;

 private:
  // Allocates a PoseBundle of length equal to the concatenation of all inputs.
  // After binding to `this`, this method has the appropriate signature for
  // an output port allocation function.
  std::unique_ptr<AbstractValue> AllocateOutputValue() const;

  enum PoseInputType {
    kUnknown = 0,
    kSinglePose = 1,
    kSingleVelocity = 2,
    kBundle = 3,
  };

  struct InputRecord {
    PoseInputType type{kUnknown};
    int num_poses{0};
    // name is only valid if type is kSingle{Pose, Velocity} or kBundle.
    std::string name{};
    // model_instance_id is only valid if type is kSingle{Pose, Velocity}.
    int model_instance_id{-1};
  };

  // Returns an InputRecord for a generic single pose input.
  static InputRecord MakeSinglePoseInputRecord(const std::string& name,
                                               int model_instance_id);

  // Returns an InputRecord for a generic single velocity input.
  static InputRecord MakeSingleVelocityInputRecord(const std::string& name,
                                                   int model_instance_id);

  // Returns an InputRecord for a generic PoseBundle input.
  static InputRecord MakePoseBundleInputRecord(const std::string& bundle_name,
                                               int num_poses);

  // Returns the total number of poses from all inputs.
  int CountNumPoses() const;

  // System<T> override.
  PoseAggregator<AutoDiffXd>* DoToAutoDiffXd() const override;

  // The type, size, and source of each input port.
  std::vector<InputRecord> input_records_;
};

}  // namespace rendering
}  // namespace systems
}  // namespace drake
