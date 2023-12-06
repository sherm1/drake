#include "drake/multibody/topology/multibody_graph.h"

#include <fmt/format.h>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"

namespace drake {
namespace multibody {
namespace internal {

MultibodyGraph::MultibodyGraph(const MultibodyGraph&) = default;
MultibodyGraph& MultibodyGraph::operator=(const MultibodyGraph&) = default;
MultibodyGraph::MultibodyGraph(MultibodyGraph&&) = default;
MultibodyGraph& MultibodyGraph::operator=(MultibodyGraph&&) = default;

MultibodyGraph::MultibodyGraph() {
  RegisterJointType(weld_type_name());
  // Verify invariant promised to users in the documentation.
  DRAKE_DEMAND(joint_type_name_to_index_[weld_type_name()] ==
               JointTypeIndex(0));
}

LinkIndex MultibodyGraph::AddLink(const std::string& link_name,
                                  ModelInstanceIndex model_instance) {
  DRAKE_DEMAND(model_instance.is_valid());

  // Reject the usage of the "world" model instance for any other links but the
  // world.
  if (num_links() > 0 && model_instance == world_model_instance()) {
    const std::string msg = fmt::format(
        "AddLink(): Model instance index = {} is reserved for the world link. "
        " link_index = 0, named '{}'",
        world_model_instance(), world_link_name());
    throw std::runtime_error(msg);
  }

  // Reject duplicate link name.
  if (HasLinkNamed(link_name, model_instance)) {
    throw std::runtime_error("AddLink(): Duplicate link name '" + link_name +
                             "'");
  }
  // next available
  const LinkIndex link_index(num_links());
  // provide fast name lookup
  link_name_to_index_.insert({link_name, link_index});

  // Can't use emplace_back below because the constructor is private.
  links_.push_back(Link(link_index, link_name, model_instance));

  return link_index;
}

bool MultibodyGraph::HasLinkNamed(const std::string& name,
                                  ModelInstanceIndex model_instance) const {
  DRAKE_DEMAND(model_instance.is_valid());

  // Search linearly on the assumption that we won't often have lots of
  // links with the same name in different model instances.  If this turns
  // out to be incorrect we can switch to a different data structure.
  const auto range = link_name_to_index_.equal_range(name);
  for (auto it = range.first; it != range.second; ++it) {
    if (get_link(it->second).model_instance() == model_instance) {
      return true;
    }
  }
  return false;
}

bool MultibodyGraph::HasJointNamed(const std::string& name,
                                   ModelInstanceIndex model_instance) const {
  DRAKE_DEMAND(model_instance.is_valid());

  // Search linearly on the assumption that we won't often have lots of
  // joints with the same name in different model instances.  If this turns
  // out to be incorrect we can switch to a different data structure.
  const auto range = joint_name_to_index_.equal_range(name);
  for (auto it = range.first; it != range.second; ++it) {
    if (get_joint(it->second).model_instance() == model_instance) {
      return true;
    }
  }
  return false;
}

const std::string& MultibodyGraph::world_link_name() const {
  if (links_.empty())
    throw std::runtime_error(
        "get_world_link_name(): you can't call this until you have called "
        "AddLink() at least once -- the first link is World.");
  return links_[0].name();
}

const MultibodyGraph::Link& MultibodyGraph::world_link() const {
  if (links_.empty())
    throw std::runtime_error(
        "world_link(): you can't call this until you have called "
        "AddLink() at least once -- the first link is 'world'.");
  return links_[0];
}

JointIndex MultibodyGraph::AddJoint(const std::string& name,
                                    ModelInstanceIndex model_instance,
                                    const std::string& type,
                                    LinkIndex parent_link_index,
                                    LinkIndex child_link_index) {
  DRAKE_DEMAND(model_instance.is_valid());

  // Reject duplicate joint name.
  if (HasJointNamed(name, model_instance)) {
    throw std::runtime_error("AddJoint(): Duplicate joint name '" + name +
                             "'.");
  }

  const JointTypeIndex type_index = GetJointTypeIndex(type);
  if (!type_index.is_valid()) {
    throw std::runtime_error("AddJoint(): Unrecognized type '" + type +
                             "' for joint '" + name + "'.");
  }

  // Verify we are connecting links within the graph.
  if (!(parent_link_index.is_valid() && parent_link_index < num_links())) {
    throw std::runtime_error("AddJoint(): parent link index for joint '" +
                             name + "' is invalid.");
  }
  if (!(child_link_index.is_valid() && child_link_index < num_links())) {
    throw std::runtime_error("AddJoint(): child link index for joint '" + name +
                             "' is invalid.");
  }

  // next available index.
  const JointIndex joint_index(num_joints());
  auto [map_iter, inserted] = links_to_joint_.insert(
      {{parent_link_index, child_link_index}, joint_index});
  if (!inserted) {
    auto existing_joint_index = map_iter->second;
    const auto& existing_joint = get_joint(existing_joint_index);
    const auto& existing_parent = get_link(existing_joint.parent_link());
    const auto& existing_child = get_link(existing_joint.child_link());
    const auto& new_parent = get_link(parent_link_index);
    const auto& new_child = get_link(child_link_index);
    throw std::runtime_error(
        "This MultibodyGraph already has a joint '" + existing_joint.name() +
        "' connecting '" + existing_parent.name() +
        "' to '" + existing_child.name() +
        "'. Therefore adding joint '" + name +
        "' connecting '" + new_parent.name() + "' to '" + new_child.name() +
        "' is not allowed.");
  }

  // provide fast name lookup.
  joint_name_to_index_.insert({name, joint_index});

  // Can't use emplace_back below because the constructor is private.
  joints_.push_back(Joint(name, model_instance, type_index, parent_link_index,
                          child_link_index));

  // Connect the graph.
  get_mutable_link(parent_link_index).add_joint(joint_index);
  get_mutable_link(child_link_index).add_joint(joint_index);

  return joint_index;
}

int MultibodyGraph::num_joint_types() const {
  return static_cast<int>(joint_type_name_to_index_.size());
}

int MultibodyGraph::num_links() const {
  return static_cast<int>(links_.size());
}

int MultibodyGraph::num_joints() const {
  return static_cast<int>(joints_.size());
}

const MultibodyGraph::Link& MultibodyGraph::get_link(LinkIndex index) const {
  DRAKE_THROW_UNLESS(index < num_links());
  return links_[index];
}

MultibodyGraph::Link& MultibodyGraph::get_mutable_link(LinkIndex link_index) {
  return links_[link_index];
}

const MultibodyGraph::Joint& MultibodyGraph::get_joint(JointIndex index) const {
  DRAKE_THROW_UNLESS(index < num_joints());
  return joints_[index];
}

JointTypeIndex MultibodyGraph::RegisterJointType(
    const std::string& joint_type_name) {
  // Reject duplicate type name.
  const auto it = joint_type_name_to_index_.find(joint_type_name);
  if (it != joint_type_name_to_index_.end())
    throw std::runtime_error(fmt::format(
        "RegisterJointType(): Duplicate joint type: '{}'.", joint_type_name));
  const JointTypeIndex joint_type_index(num_joint_types());
  joint_type_name_to_index_[joint_type_name] = joint_type_index;
  return joint_type_index;
}

bool MultibodyGraph::IsJointTypeRegistered(
    const std::string& joint_type_name) const {
  const auto it = joint_type_name_to_index_.find(joint_type_name);
  return it != joint_type_name_to_index_.end();
}

JointTypeIndex MultibodyGraph::GetJointTypeIndex(
    const std::string& joint_type_name) const {
  const auto it = joint_type_name_to_index_.find(joint_type_name);
  return it == joint_type_name_to_index_.end() ? JointTypeIndex() : it->second;
}

std::vector<std::set<LinkIndex>> MultibodyGraph::FindSubgraphsOfWeldedLinks()
    const {
  std::vector<bool> visited(num_links(), false);
  std::vector<std::set<LinkIndex>> subgraphs;

  // Reserve the maximum possible number of subgraphs (that is, when each link
  // forms its own subgraph) in advance in order to avoid reallocation in the
  // std::vector "subgraphs" which would cause the invalidation of references as
  // we recursively fill it in.
  subgraphs.reserve(num_links());

  // The first link visited is the "world" (link_index = 0), and therefore
  // subgraphs[0] corresponds to the subgraphs of all links welded to the
  // world.
  for (const auto& link : links_) {
    if (!visited[link.index()]) {
      // If `link` was not visited yet, we create an subgraph for it.
      subgraphs.push_back(std::set<LinkIndex>{link.index()});

      // We build the subgraph to which `link` belongs by recursively traversing
      // the sub-graph it belongs to.
      std::set<LinkIndex>& link_subgraph = subgraphs.back();

      // Thus far `link` forms its own subgraph. Find if other links belong to
      // this subgraph by recursively traversing the sub-graph of welded joints
      // connected to `link`.
      FindSubgraphsOfWeldedLinksRecurse(link, &link_subgraph, &subgraphs,
                                        &visited);
    }
  }
  return subgraphs;
}

void MultibodyGraph::FindSubgraphsOfWeldedLinksRecurse(
    const Link& parent_link, std::set<LinkIndex>* parent_subgraph,
    std::vector<std::set<LinkIndex>>* subgraphs,
    std::vector<bool>* visited) const {
  // Mark parent_link as visited in order to detect loops.
  visited->at(parent_link.index()) = true;

  // Scan each sibling link.
  for (JointIndex joint_index : parent_link.joints()) {
    const Joint& joint = get_joint(joint_index);
    const LinkIndex sibling_index = joint.parent_link() == parent_link.index()
                                        ? joint.child_link()
                                        : joint.parent_link();

    // If already visited continue with the next joint.
    if (visited->at(sibling_index)) continue;

    const Link& sibling = get_link(sibling_index);
    if (joint.type_index() == weld_type_index()) {
      // Welded to parent_link, add it to parent_subgraph.
      parent_subgraph->insert(sibling_index);
      FindSubgraphsOfWeldedLinksRecurse(sibling, parent_subgraph, subgraphs,
                                        visited);
    } else {
      // Disconnected (non-welded) from parent_subgraph. Create its own new
      // subgraph and continue the recursion from "sibling" with its new
      // subgraph "sibling_subgraph".
      subgraphs->push_back(std::set<LinkIndex>{sibling_index});
      std::set<LinkIndex>& sibling_subgraph = subgraphs->back();
      FindSubgraphsOfWeldedLinksRecurse(sibling, &sibling_subgraph, subgraphs,
                                        visited);
    }
  }
}

std::set<LinkIndex> MultibodyGraph::FindLinksWeldedTo(
    LinkIndex link_index) const {
  DRAKE_THROW_UNLESS(link_index.is_valid() && link_index < num_links());

  // TODO(amcastro-tri): Notice that "subgraphs" will get compute with every
  // call to FindLinksWeldedTo(). Consider storing this for subsequent calls if
  // it becomes a performance bottleneck.
  const std::vector<std::set<LinkIndex>> subgraphs =
      FindSubgraphsOfWeldedLinks();

  // Find subgraph that contains this link_index.
  // TODO(amcastro-tri): Consider storing within Link the subgraph it belongs to
  //  if performance becomes an issue.
  auto predicate = [link_index](auto& subgraph) {
    return subgraph.count(link_index) > 0;
  };
  auto subgraph_iter =
      std::find_if(subgraphs.begin(), subgraphs.end(), predicate);

  // If link_index is a valid index to a link in this graph, then it MUST belong
  // to one of the subgraphs. We verify this explicitly.
  DRAKE_DEMAND(subgraph_iter != subgraphs.end());

  return *subgraph_iter;
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
