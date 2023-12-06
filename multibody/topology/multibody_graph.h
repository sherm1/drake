#pragma once

#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/sorted_pair.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {
namespace internal {

/* Type used to identify joint types. */
using JointTypeIndex = TypeSafeIndex<class JointTypeTag>;

/* Defines a multibody graph consisting of links interconnected by joints.
The graph is defined by a sequence of calls to AddLink() and AddJoint(). Anytime
during the lifetime of the graph, a user can ask graph specific questions such
as how links are connected, by which joints or even perform more complex
queries such as what set of links are welded together. */
// TODO(amcastro-tri): consider moving outside internal:: and available to users
// of MBP, towards #11307.
class MultibodyGraph {
 public:
  // We can't use the DRAKE_DEFAULT_COPY... macro for this class; see below for
  // the declarations of the copy, assign, and move functions.

  // The classes are defined later on in this header file.
  class Link;
  class Joint;

  MultibodyGraph();

  // Because we use vectors of forward-declared classes (e.g., vector<Link>) as
  // member fields, we can only _declare_ our default copyable functions here;
  // we must _define_ them only after MbG::Link and MbG::Joint are defined,
  // i.e., in our *.cc file.
  MultibodyGraph(const MultibodyGraph&);
  MultibodyGraph& operator=(const MultibodyGraph&);
  MultibodyGraph(MultibodyGraph&&);
  MultibodyGraph& operator=(MultibodyGraph&&);

  /* Add a new Link to the graph.
  @param[in] name
    The unique name of the new link in the particular `model_instance`. Several
    links can have the same name within a %MultibodyGraph however, their name
    within their model instance must be unique.
  @param[in] model_instance
    The model instance to which this link belongs, see @ref model_instance.
  @note The first call to AddLink() defines the "world" link's `name`. For this
  first call `model_instance` must be world_model_instance().
  @returns The unique LinkIndex for the added joint in the graph. */
  LinkIndex AddLink(const std::string& name, ModelInstanceIndex model_instance);

  /* Add a new Joint to the graph.
  @param[in] name
    The unique name of the new Joint in the particular `model_instance`. Several
    joints can have the same name within a %MultibodyGraph however, their name
    within their model instance must be unique.
  @param[in] model_instance
    The model instance to which this joint belongs, see @ref model_instance.
  @param[in] type
    A string designating the type of this joint, such as "revolute" or
    "ball". This must be chosen from the set of joint types previously
    registered with calls to RegisterJointType().
  @param[in] parent_link_index
    This must be the index of a link previously obtained with a call to
    AddLink(), or it must be the designated index for the world link
    (world_index()).
  @param[in] child_link_index
    This must be the index of a link previously obtained with a call to
    AddLink(), or it must be the designated index for the world link
    (world_index()).
  @returns The unique JointIndex for the added joint in the graph.
  @throws std::exception iff `name` is duplicated within `model_instance`.
  @throws std::exception iff `type` has not been registered with
  RegisterJointType().
  @throws std::exception iff `parent_link_index` or `child_link_index` are
  not valid link indexes for `this` graph. */
  JointIndex AddJoint(const std::string& name,
                      ModelInstanceIndex model_instance,
                      const std::string& type, LinkIndex parent_link_index,
                      LinkIndex child_link_index);

  /* Returns the link that corresponds to the world. This link added via the
  first call to AddLink().
  @throws std::exception iff AddLink() was not called even once yet. */
  const Link& world_link() const;

  /* Returns the name we recognize as the World (or Ground) link. This is
  the name that was provided in the first AddLink() call.
  In Drake, MultibodyPlant names it the "world".
  @throws std::exception iff AddLink() was not called even once yet. */
  const std::string& world_link_name() const;

  /* Returns the unique index that identifies the "weld" joint type (always
  zero).
  @note The SDF format calls this type a "fixed" joint. */
  static JointTypeIndex weld_type_index() { return JointTypeIndex(0); }

  /* Returns the unique name reserved to identify weld joints (always "weld").
   */
  static std::string weld_type_name() { return "weld"; }

  /* Register a joint type by name.
  MultibodyGraph() registers "weld" at construction as Drake reserves this name
  to identify weld joints. "weld" joint type has index `weld_type_index()`.
  @param[in] joint_type_name
    A unique string identifying a joint type, such as "pin" or "prismatic".
  @throws std::exception if `joint_type_name` already identifies a
  previously registered joint type.
  @retval JointTypeIndex Index uniquely identifying the new joint type. */
  JointTypeIndex RegisterJointType(const std::string& joint_type_name);

  /* @returns `true` iff the given `joint_type_name` was previously registered
  via a call to RegisterJointType(), or iff it equals weld_type_name(). */
  bool IsJointTypeRegistered(const std::string& joint_type_name) const;

  /* Returns the number of registered joint types. */
  int num_joint_types() const;

  /* Returns the number of links, including all added links, and the world
  link.
  @see AddLink(), world_index(), world_link_name(). */
  int num_links() const;

  /** Returns the number joints added with AddJoint(). */
  int num_joints() const;

  /* Gets a Link by index. The world link has index world_index().
  @throws std::exception iff `index` does not correspond to a link in this
  graph. */
  const Link& get_link(LinkIndex index) const;

  /* Gets a Joint by index.
  @throws std::exception iff `index` does not correspond to a joint in this
  graph. */
  const Joint& get_joint(JointIndex index) const;

  /* @returns `true` if a link named `name` was added to `model_instance`.
  @see AddLink().
  @throws std::exception if `model_instance` is not a valid index. */
  bool HasLinkNamed(const std::string& name,
                    ModelInstanceIndex model_instance) const;

  /* @returns `true` if a joint named `name` was added to `model_instance`.
  @see AddLink().
  @throws std::exception if `model_instance` is not a valid index. */
  bool HasJointNamed(const std::string& name,
                     ModelInstanceIndex model_instance) const;

  /* This method partitions the %MultibodyGraph into sub-graphs such that (a)
  every link is in one and only one sub-graph, and (b) two links are in the
  same sub-graph iff there is a path between them which includes only weld
  joints (see weld_type_name()). Each sub-graph of welded links is represented
  as a set of link indices. By definition, these sub-graphs will be disconnected
  by any non-weld joint between two links. The first sub-graph will have all of
  the links welded to the world, including the world link itself; all
  subsequent sub-graphs will be in no particular order. A few more notes:
    - Each link in the %MultibodyGraph is included in one set and one set only.
    - The maximum number of returned sub-graphs equals the number of links in
      the graph (num_links()). This corresponds to a graph with no weld joints.
    - The world link is also included in a set of welded links, and this set is
      element zero in the returned vector.
    - The minimum number of sub-graphs is one. This corresponds to a graph with
      all links welded to the world. */
  std::vector<std::set<LinkIndex>> FindSubgraphsOfWeldedLinks() const;

  /* Returns all links that are transitively welded, or rigidly affixed, to
  `link_index`, per these two definitions:
    1. A link is always considered welded to itself.
    2. Two unique links are considered welded together exclusively by the
       presence of a weld joint, not by other constructs such as constraints.
  Therefore, if `link_index` is a valid index to a Link in this graph, then the
  return vector will always contain at least one entry storing `link_index`.
  @throws std::exception iff `link_index` does not correspond to a link in this
  graph. */
  std::set<LinkIndex> FindLinksWeldedTo(LinkIndex link_index) const;

 private:
  // Finds the assigned index for a joint type from the type name. Returns an
  // invalid index if `joint_type_name` was not previously registered with a
  // call to RegisterJointType().
  JointTypeIndex GetJointTypeIndex(const std::string& joint_type_name) const;

  Link& get_mutable_link(LinkIndex link_index);

  // Recursive helper method for FindSubgraphsOfWeldedLinks().
  // The first thing this helper does is to mark `parent_link` as "visited" in
  // the output array `visited`.
  // Next, it scans the sibling links connected to `parent_index` by a joint.
  // If the sibling was already visited, it moves on to the next sibling, if
  // any. For a sibling visited for the first time there are two options:
  //   1) if it is connected by a weld joint, it gets added to parent_subgraph.
  //      Recursion continues starting with parent_index = sibling and the
  //      parent sub-graph. Otherwise,
  //   2) a new sub-graph is created for the sibling and gets added to the
  //      list of all `subgraphs`. Recursion continues starting with
  //      parent_index = sibling and the new sub-graph for this sibling.
  void FindSubgraphsOfWeldedLinksRecurse(
      const Link& parent_link, std::set<LinkIndex>* parent_subgraph,
      std::vector<std::set<LinkIndex>>* subgraphs,
      std::vector<bool>* visited) const;

  // links_ includes the world link at world_index() with name
  // world_link_name().
  std::vector<Link> links_;
  std::vector<Joint> joints_;

  std::unordered_map<std::string, JointTypeIndex> joint_type_name_to_index_;

  // Map used to detect redundant joints.
  using LinksKey = SortedPair<LinkIndex>;
  std::unordered_map<LinksKey, JointIndex> links_to_joint_;

  // The xxx_name_to_index_ structures are multimaps because
  // links/joints/actuators/etc may appear with the same name in different
  // model instances. The index values are still unique across the graph.
  std::unordered_multimap<std::string, LinkIndex> link_name_to_index_;
  std::unordered_multimap<std::string, JointIndex> joint_name_to_index_;
};

class MultibodyGraph::Link {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Link)

  /* @returns its unique index in the graph. */
  LinkIndex index() const { return index_; }

  /* @returns its model instance. */
  ModelInstanceIndex model_instance() const { return model_instance_; }

  /* @returns its name, unique within model_instance(). */
  const std::string& name() const { return name_; }

  /* Returns the total number of joints that have `this` link as either the
  parent or child link in a Joint. */
  int num_joints() const;

  /* @returns all the joints that connect to `this` link. */
  const std::vector<JointIndex>& joints() const { return joints_; }

 private:
  // Restrict construction and modification to only MultibodyGraph.
  friend class MultibodyGraph;

  Link(LinkIndex index, const std::string& name,
       ModelInstanceIndex model_instance)
      : index_(index), name_(name), model_instance_(model_instance) {}

  // Notes that this link is connected by `joint`.
  void add_joint(JointIndex joint) { joints_.push_back(joint); }

  LinkIndex index_;
  std::string name_;
  ModelInstanceIndex model_instance_;

  // All joints connecting this link. The union (in no particular order) of
  // joints_as_parent_ and joints_as_child_.
  std::vector<JointIndex> joints_;
};

class MultibodyGraph::Joint {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Joint)

  ModelInstanceIndex model_instance() const { return model_instance_; }

  const std::string& name() const { return name_; }

  LinkIndex parent_link() const { return parent_link_index_; }
  LinkIndex child_link() const { return child_link_index_; }

  JointTypeIndex type_index() const { return type_index_; }

 private:
  // Restrict construction and modification to only MultibodyGraph.
  friend class MultibodyGraph;

  Joint(const std::string& name, ModelInstanceIndex model_instance,
        JointTypeIndex joint_type_index, LinkIndex parent_link_index,
        LinkIndex child_link_index)
      : name_(name),
        model_instance_(model_instance),
        type_index_(joint_type_index),
        parent_link_index_(parent_link_index),
        child_link_index_(child_link_index) {}

  std::string name_;
  ModelInstanceIndex model_instance_;
  JointTypeIndex type_index_;
  LinkIndex parent_link_index_;
  LinkIndex child_link_index_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
