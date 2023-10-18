#pragma once

/** @file This is the file to include for working with LinkJointGraph. It
includes several other files which should not be included directly. */

// Subsidiary files barf if they don't see this define.
#define DRAKE_GRAPH_INCLUDED

/* Terminology note: we use "Link" here to mean what MultibodyPlant calls a
"Body". For purposes of graph analysis we want to clearly distinguish the
input graph from the spanning forest model we're going to build. To do that we
will use Link/Joint for the nodes and parent/child directed edges of the user's
input structure, and Body/Mobilizer (a.k.a. MobilizedBody) for the nodes and
inboard/outboard directed edges of the generated forest. */
// TODO(sherm1) Consider switching to "Link" in MultibodyPlant for clarity
//  and consistency.

// Shared definitions.
#include "drake/multibody/topology/link_joint_graph_defs.h"

// class LinkJointGraph
#include "drake/multibody/topology/link_joint_graph.h"

// class LinkJointGraph::Link
#include "drake/multibody/topology/link_joint_graph_link.h"

