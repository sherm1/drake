#include "drake/multibody/topology/multibody_graph.h"

#include <string>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace multibody {
namespace internal {

const char kRevoluteType[] = "revolute";
const char kPrismaticType[] = "prismatic";

// Arbitrary world name for testing.
const char kWorldLinkName[] = "DefaultWorldLinkName";

// Test a straightforward serial chain of links connected by
// revolute joints: world->link1->link2->link3->link4->link5.
// We perform a number of sanity checks on the provided API.
GTEST_TEST(MultibodyGraph, SerialChain) {
  MultibodyGraph graph;
  EXPECT_EQ(graph.num_joint_types(), 1);  // "weld" joint thus far.
  graph.RegisterJointType(kRevoluteType);
  EXPECT_EQ(graph.num_joint_types(), 2);  // "weld" and "revolute".

  // Verify what joint types were registered.
  EXPECT_TRUE(graph.IsJointTypeRegistered(kRevoluteType));
  EXPECT_FALSE(graph.IsJointTypeRegistered(kPrismaticType));

  // The first link added defines the world's name and model instance.
  // We'll verify their values below.
  graph.AddLink(kWorldLinkName, world_model_instance());

  // We'll add the chain to this model.
  const ModelInstanceIndex model_instance(5);

  // We cannot register to the world model instance, unless it's the first call
  // to AddLink().
  DRAKE_EXPECT_THROWS_MESSAGE(
      graph.AddLink("InvalidLink", world_model_instance()),
      fmt::format("AddLink\\(\\): Model instance index = {}.*",
                  world_model_instance()));

  LinkIndex parent = graph.AddLink("link1", model_instance);
  graph.AddJoint("pin1", model_instance, kRevoluteType, world_index(), parent);
  for (int i = 2; i <= 5; ++i) {
    LinkIndex child = graph.AddLink("link" + std::to_string(i), model_instance);
    graph.AddJoint("pin" + std::to_string(i), model_instance, kRevoluteType,
                   parent, child);
    parent = child;
  }

  // We cannot duplicate the name of a link or joint.
  DRAKE_EXPECT_THROWS_MESSAGE(graph.AddLink("link3", model_instance),
                              "AddLink\\(\\): Duplicate link name.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      graph.AddJoint("pin3", model_instance, kRevoluteType, LinkIndex(1),
                     LinkIndex(2)),
      "AddJoint\\(\\): Duplicate joint name.*");

  // We cannot add a redundant joint.
  DRAKE_EXPECT_THROWS_MESSAGE(
      graph.AddJoint("other", model_instance, kRevoluteType, LinkIndex(1),
                     LinkIndex(2)),
      "This MultibodyGraph already has a joint 'pin2' connecting 'link1'"
      " to 'link2'. Therefore adding joint 'other' connecting 'link1' to"
      " 'link2' is not allowed.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      graph.AddJoint("reverse", model_instance, kRevoluteType, LinkIndex(2),
                     LinkIndex(1)),
      "This MultibodyGraph already has a joint 'pin2' connecting 'link1'"
      " to 'link2'. Therefore adding joint 'reverse' connecting 'link2' to"
      " 'link1' is not allowed.");

  // We cannot add an unregistered joint type.
  DRAKE_EXPECT_THROWS_MESSAGE(graph.AddJoint("screw1", model_instance, "screw",
                                             LinkIndex(1), LinkIndex(2)),
                              "AddJoint\\(\\): Unrecognized type.*");

  // Invalid parent/child link throws.
  DRAKE_EXPECT_THROWS_MESSAGE(
      graph.AddJoint("another_pin", model_instance, kRevoluteType, LinkIndex(1),
                     LinkIndex(9)),
      "AddJoint\\(\\): child link index for joint '.*' is invalid.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      graph.AddJoint("another_pin", model_instance, kRevoluteType, LinkIndex(9),
                     LinkIndex(1)),
      "AddJoint\\(\\): parent link index for joint '.*' is invalid.");

  // Verify the world's name and model instance are registered correctly.
  EXPECT_EQ(graph.world_link_name(), kWorldLinkName);
  EXPECT_EQ(graph.world_link().model_instance(), world_model_instance());
  // Link "0" is always the world.
  EXPECT_EQ(graph.get_link(LinkIndex(0)).name(), kWorldLinkName);
  EXPECT_EQ(graph.get_link(LinkIndex(0)).model_instance(),
            world_model_instance());

  // Sanity check sizes.
  EXPECT_EQ(graph.num_links(), 6);  // this includes the world link.
  EXPECT_EQ(graph.num_joints(), 5);

  // Verify we can get links/joints.
  EXPECT_EQ(graph.get_link(LinkIndex(3)).name(), "link3");
  EXPECT_EQ(graph.get_joint(JointIndex(3)).name(), "pin4");
  DRAKE_EXPECT_THROWS_MESSAGE(graph.get_link(LinkIndex(9)),
                              ".*index < num_links\\(\\).*");
  DRAKE_EXPECT_THROWS_MESSAGE(graph.get_joint(JointIndex(9)),
                              ".*index < num_joints\\(\\).*");

  // Verify we can query if a link/joint is in the graph.
  const ModelInstanceIndex kInvalidModelInstance(666);
  EXPECT_TRUE(graph.HasLinkNamed("link3", model_instance));
  EXPECT_FALSE(graph.HasLinkNamed("link3", kInvalidModelInstance));
  EXPECT_FALSE(graph.HasLinkNamed("invalid_link_name", model_instance));
  EXPECT_TRUE(graph.HasJointNamed("pin3", model_instance));
  EXPECT_FALSE(graph.HasJointNamed("pin3", kInvalidModelInstance));
  EXPECT_FALSE(graph.HasJointNamed("invalid_joint_name", model_instance));
}

// We build a model containing a number of kinematic loops and subgraphs of
// welded links.
//
// subgraph A (forms closed loop):
//  - WeldJoint(1, 13)
//  - WeldJoint(1, 4)
//  - WeldJoint(4, 13)
//
// subgraph B (forms closed loop):
//  - WeldJoint(6, 10)
//  - WeldJoint(6, 8)
//  - WeldJoint(8, 10)
//
// subgraph C (the "world" subgraph):
//  - WeldJoint(5, 7)
//  - WeldJoint(5, 12)
//
// Non-weld joints kinematic loop:
//  - PrimaticJoint(2, 11)
//  - PrimaticJoint(2, 7)
//  - RevoluteJoint(7, 11)
//
// Additionally we have the following non-weld joints:
//  - RevoluteJoint(3, 13): connects link 3 to subgraph A.
//  - PrimaticJoint(1, 10): connects subgraph A and B.
//
// Therefore we expect the following subgraphs, in no particular order, but with
// the "world" subgraph first:
//   {0, 5, 7, 12}, {1, 4, 13}, {6, 8, 10}, {3}, {9}, {2}, {11}.
GTEST_TEST(MultibodyGraph, Weldedsubgraphs) {
  MultibodyGraph graph;
  graph.RegisterJointType(kRevoluteType);
  graph.RegisterJointType(kPrismaticType);
  EXPECT_EQ(graph.num_joint_types(), 3);  // weld, revolute and prismatic.

  // The first link added defines the world's name and model instance.
  graph.AddLink(kWorldLinkName, world_model_instance());

  // We'll add links and joints to this model instance.
  const ModelInstanceIndex model_instance(5);

  // Define the model.
  for (int i = 1; i <= 13; ++i) {
    graph.AddLink("link" + std::to_string(i), model_instance);
  }

  // Add joints.
  int j = 0;

  // subgraph A: formed by links 1, 4, 13.
  graph.AddJoint("joint" + std::to_string(j++), model_instance,
                 graph.weld_type_name(), LinkIndex(1), LinkIndex(13));
  graph.AddJoint("joint" + std::to_string(j++), model_instance,
                 graph.weld_type_name(), LinkIndex(4), LinkIndex(1));
  graph.AddJoint("joint" + std::to_string(j++), model_instance,
                 graph.weld_type_name(), LinkIndex(13), LinkIndex(4));

  // Link 3 connects to subgraph A via a revolute joint.
  graph.AddJoint("joint" + std::to_string(j++), model_instance, kRevoluteType,
                 LinkIndex(3), LinkIndex(13));

  // subgraph B: formed by links 8, 6, 10.
  graph.AddJoint("joint" + std::to_string(j++), model_instance,
                 graph.weld_type_name(), LinkIndex(10), LinkIndex(6));
  graph.AddJoint("joint" + std::to_string(j++), model_instance,
                 graph.weld_type_name(), LinkIndex(10), LinkIndex(8));
  graph.AddJoint("joint" + std::to_string(j++), model_instance,
                 graph.weld_type_name(), LinkIndex(6), LinkIndex(8));

  // Link 1 (in subgraph A) and link 10 (in subgraph B) connect through a
  // prismatic joint.
  graph.AddJoint("joint" + std::to_string(j++), model_instance, kPrismaticType,
                 LinkIndex(1), LinkIndex(10));

  // Closed kinematic loop of non-weld joints.
  graph.AddJoint("joint" + std::to_string(j++), model_instance, kPrismaticType,
                 LinkIndex(2), LinkIndex(11));
  graph.AddJoint("joint" + std::to_string(j++), model_instance, kPrismaticType,
                 LinkIndex(7), LinkIndex(2));
  graph.AddJoint("joint" + std::to_string(j++), model_instance, kRevoluteType,
                 LinkIndex(7), LinkIndex(11));

  // subgraph C: formed by links 5, 7, 12.
  graph.AddJoint("joint" + std::to_string(j++), model_instance,
                 graph.weld_type_name(), LinkIndex(5), LinkIndex(7));
  graph.AddJoint("joint" + std::to_string(j++), model_instance,
                 graph.weld_type_name(), LinkIndex(5), world_index());
  graph.AddJoint("joint" + std::to_string(j++), model_instance,
                 graph.weld_type_name(), LinkIndex(12), LinkIndex(5));

  EXPECT_EQ(graph.num_links(), 14);  // this includes the world link.

  const std::vector<std::set<LinkIndex>> welded_subgraphs =
      graph.FindSubgraphsOfWeldedLinks();

  // Verify number of expected subgraphs.
  EXPECT_EQ(welded_subgraphs.size(), 7);

  // The first subgraph must contain the world.
  const std::set<LinkIndex> world_subgraph = welded_subgraphs[0];
  EXPECT_EQ(world_subgraph.count(world_index()), 1);

  // Build the expected set of subgraphs.
  std::set<std::set<LinkIndex>> expected_subgraphs;
  //   {0, 5, 7, 12}, {1, 4, 13}, {6, 8, 10}, {3}, {9}, {2}, {11}.
  const std::set<LinkIndex>& expected_world_subgraph =
      *expected_subgraphs
           .insert({LinkIndex(0), LinkIndex(5), LinkIndex(7), LinkIndex(12)})
           .first;
  const std::set<LinkIndex>& expected_subgraphA =
      *expected_subgraphs.insert({LinkIndex(1), LinkIndex(4), LinkIndex(13)})
           .first;
  const std::set<LinkIndex>& expected_subgraphB =
      *expected_subgraphs.insert({LinkIndex(6), LinkIndex(8), LinkIndex(10)})
           .first;
  expected_subgraphs.insert({LinkIndex(3)});
  expected_subgraphs.insert({LinkIndex(9)});
  expected_subgraphs.insert({LinkIndex(2)});
  expected_subgraphs.insert({LinkIndex(11)});

  // We do expect the first subgraph to correspond to the set of links welded
  // to the world.
  EXPECT_EQ(world_subgraph, expected_world_subgraph);

  // In order to compare the computed list of welded links against the expected
  // list, irrespective of the ordering in the computed list, we first convert
  // the computed subgraphs to a set.
  const std::set<std::set<LinkIndex>> welded_subgraphs_set(
      welded_subgraphs.begin(), welded_subgraphs.end());
  EXPECT_EQ(welded_subgraphs_set, expected_subgraphs);

  // Verify we can query the list of links welded to a particular link.
  EXPECT_EQ(graph.FindLinksWeldedTo(LinkIndex(9)).size(), 1);
  EXPECT_EQ(graph.FindLinksWeldedTo(LinkIndex(11)).size(), 1);
  EXPECT_EQ(graph.FindLinksWeldedTo(LinkIndex(4)), expected_subgraphA);
  EXPECT_EQ(graph.FindLinksWeldedTo(LinkIndex(13)), expected_subgraphA);
  EXPECT_EQ(graph.FindLinksWeldedTo(LinkIndex(10)), expected_subgraphB);
  EXPECT_EQ(graph.FindLinksWeldedTo(LinkIndex(6)), expected_subgraphB);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
